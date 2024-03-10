#!/usr/bin/env python3
from cmath import e
import queue
import numpy as np
import random as rm
import time
import rospy
import copy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, PointCloud
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray
import message_filters

np.set_printoptions(precision=8, suppress=True)

# 왼쪽 카메라 파라미터
left_ALPHA = np.radians(25)
left_BETA = np.radians(31)
left_cam_2_ldr_tlc = [0.645, -0.15, -0.43] # 카메라 -> 라이다의 벡터 / 카메라 좌표계 기준
left_camera_matrix = np.array([[420.90359778 ,  0.    ,     308.76324662 , 0. ],
                                [  0.   ,      550.45898138 , 220.75188643 ,0.],
                                [  0.    ,       0.        ,   1.      ,0.  ]])

# np.array([[421.09823608 ,  0.    ,     346.89054405, 0. ],
#                                 [  0.   ,      443.14282227 , 197.09642544,0.],
#                                 [  0.    ,       0.        ,   1.      ,0.  ]])

# 오른쪽 카메라 파라미터5
right_ALPHA = np.radians(25)
right_BETA = np.radians(-31)
right_cam_2_ldr_tlc = [0.645, 0.15, -0.43] # 카메라 -> 라이다의 벡터 / 카메라 좌표계 기준
right_camera_matrix = np.array([[420.84070955 ,  0.     ,    330.01143253 , 0.],
 [  0.       ,  550.07428424, 207.62107617, 0.],
 [  0.     ,      0.     ,      1.      ,0.0  ]])

# np.array([[723.91193822 ,  0.     ,    425.94406173 , 0.],
#  [  0.       ,  761.85427703, 277.42251396, 0.],
#  [  0.     ,      0.     ,      1.      ,0.0  ]])


class SensorFusion():
    def __init__(self):
        rospy.init_node('sensor_fusion')
        self.lidar_sub = message_filters.Subscriber('/laser_roi_filtered', PointCloud)
        # self.image_sub = message_filters.Subscriber("/image_example/raw", Image)
        self.image_sub = message_filters.Subscriber("/image_combined", Image)

        self.bounding_box_sub = message_filters.Subscriber('/bounding_boxes', PoseArray)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.lidar_sub, self.image_sub, self.bounding_box_sub], queue_size=10, slop=0.5, allow_headerless=True)
        self.ts.registerCallback(self.callback_fusion)

        self.sensor_fusion_centroid_point_pub = rospy.Publisher('/sensor_fusion_centroid_point', MarkerArray, queue_size=10)
        # self.sensor_fusion_points_pub = rospy.Publisher('/sensor_fusion_points', MarkerArray, queue_size=10)
        self.image_check_pub = rospy.Publisher('/sensor_fusion_image', Image, queue_size=10)

        self.left_rtlc = self.get_rtlc(alpha=left_ALPHA, beta=left_BETA, tlc=left_cam_2_ldr_tlc)
        self.right_rtlc = self.get_rtlc(alpha=right_ALPHA, beta=right_BETA, tlc=right_cam_2_ldr_tlc)

        self.lidar_msg = None
        self.img_raw_msg = None
        self.bbox_msg = None
        self.img_raw = None
        self.bridge = CvBridge()
        return
    
    def callback_fusion(self, lidar, img_raw, bbox):
        first_time = time.time()

        centroid_mrk_arry = MarkerArray()
        img_visualize = copy.deepcopy(self.bridge.imgmsg_to_cv2(img_raw, desired_encoding="bgr8"))

        left_bboxes = [[bbox.orientation.z, bbox.orientation.x, bbox.orientation.w, bbox.orientation.y] for bbox in bbox.poses if bbox.orientation.z < 640]
        right_bboxes = [[bbox.orientation.z-640, bbox.orientation.x-640, bbox.orientation.w, bbox.orientation.y] for bbox in bbox.poses if bbox.orientation.x >= 640]

        img_left = self.sensor_fusion(self.left_rtlc, left_camera_matrix, lidar, left_bboxes, centroid_mrk_arry, img_visualize[:,:640])
        img_right = self.sensor_fusion(self.right_rtlc, right_camera_matrix, lidar, right_bboxes, centroid_mrk_arry, img_visualize[:,640:])
        img_result = np.concatenate([img_left, img_right], axis=1)

        img_for_pub = self.bridge.cv2_to_imgmsg(img_visualize, "bgr8")
        img_for_pub.header.stamp = rospy.Time.now()
        self.image_check_pub.publish(img_for_pub)

        self.sensor_fusion_centroid_point_pub.publish(centroid_mrk_arry)
        print("계산 시간 : ", time.time() - first_time)
        return

    def sensor_fusion(self, rtlc, camera_matrix, lidar_msg, bboxes, centroid_mrk_arry, img_visualize):
        closest_points = [None] * len(bboxes)
        closest_dists = [np.inf] * len(bboxes)
        # bbox_centers = np.array([((bbox[0] + bbox[1])/2, (bbox[2] + bbox[3])/2) for bbox in bboxes])
        bbox_centers = np.array([((bbox[0] + bbox[1])/2, bbox[2]) for bbox in bboxes])
        # bbox 중심점을 빨간색으로 표시
        for bbox_center in bbox_centers:
            center_x, center_y = bbox_center
            img_visualize[int(center_y)-10:int(center_y)+10, int(center_x)-10:int(center_x)+10] = (0, 0, 255)

        points_world = np.array([[point.x, point.y, point.z, 1.0] for point in lidar_msg.points]).T
        points_c = camera_matrix @ (rtlc @ points_world)

        pixel_x = points_c[0,:] // points_c[2,:]
        pixel_y = points_c[1,:] // points_c[2,:]

        height, width, _ = img_visualize.shape
        valid_idx = ~np.isnan(pixel_x) & ~np.isnan(pixel_y) & (pixel_x > 0) & (pixel_x < width) & (pixel_y > 0 ) & (pixel_y < height)
        img_visualize[pixel_y[valid_idx].astype(int), pixel_x[valid_idx].astype(int)] = (0, 255, 0)

        pixel_x = pixel_x[valid_idx]
        pixel_y = pixel_y[valid_idx]
        points_world = points_world[:,valid_idx]

        for i, bbox in enumerate(bboxes):
            dist = (bbox_centers[i,0] - pixel_x)**2 + (bbox_centers[i,1] - pixel_y)**2
            
            min_dist_idx = np.argmin(dist)
            closest_points[i] = tuple(points_world[:,min_dist_idx])[:3] 
            closest_dists[i] = dist[min_dist_idx]
            # 가장 가깝다고 판단된 점을 이미지에 표시
            img_visualize[int(pixel_y[min_dist_idx])-5:int(pixel_y[min_dist_idx]) +5, int(pixel_x[min_dist_idx])-5:int(pixel_x[min_dist_idx])+5] = (255, 0, 0)

        for closest_point in closest_points:
            if closest_point is not None:
                idx = rm.randint(0, 99999)
                centroid_mrk_arry.markers.append(self.__make_marker__(closest_point, idx))
        return img_visualize
    
    # def sensor_fusion_right(self, rtlc, camera_matrix, bboxes, centroid_mrk_arry, img_visualize):
    #     closest_points = [None] * len(bboxes)
    #     closest_dists = [np.inf] * len(bboxes)
    #     bbox_centers = np.array([((bbox[0] + bbox[1])/2, (bbox[2] + bbox[3])/2) for bbox in bboxes])
        
    #     # bbox 중심점을 빨간색으로 표시
    #     for bbox_center in bbox_centers:
    #         center_x, center_y = bbox_center
    #         img_visualize[int(center_y)-10:int(center_y)+10, int(center_x)-10 :int(center_x)+10] = (0, 0, 255)

    #     points_world = np.array([[point.x, point.y, point.z, 1.0] for point in self.lidar_msg.points]).T
    #     points_c = camera_matrix @ (rtlc @ points_world)

    #     pixel_x = points_c[0,:] // points_c[2,:]
    #     pixel_y = points_c[1,:] // points_c[2,:]

    #     valid_idx = ~np.isnan(pixel_x) & ~np.isnan(pixel_y) & (pixel_x >= 0) & (pixel_x < 640) & (pixel_y >= 0 ) & (pixel_y < 360)
    #     img_visualize[pixel_y[valid_idx].astype(int), pixel_x[valid_idx].astype(int)] = (0, 255, 0)

    #     pixel_x = pixel_x[valid_idx]
    #     pixel_y = pixel_y[valid_idx]
    #     points_world = points_world[:,valid_idx]

    #     for i, bbox in enumerate(bboxes):
    #         dist = (bbox_centers[i,0] - pixel_x)**2 + (bbox_centers[i,1] - pixel_y)**2
    #         min_dist_idx = np.argmin(dist)
    #         closest_points[i] = tuple(points_world[:,min_dist_idx])[:3] 
    #         closest_dists[i] = dist[min_dist_idx]
    #         # 가장 가깝다고 판단된 점을 이미지에 표시
    #         img_visualize[int(pixel_y[min_dist_idx])-5:int(pixel_y[min_dist_idx]) +5, int(pixel_x[min_dist_idx])-5:int(pixel_x[min_dist_idx])+5] = (255, 0, 0)

    #     for closest_point in closest_points:
    #         if closest_point is not None:
    #             idx = rm.randint(0, 99999)
    #             centroid_mrk_arry.markers.append(self.__make_marker__(closest_point, idx))
    #     return img_visualize
    
    # def sensor_fusion(self, rtlc, camera_matrix, bboxes, centroid_mrk_arry, img_visualize):
    #     for bbox in bboxes:
    #         bbox_xmax, bbox_xmin, bbox_ymax, bbox_ymin = bbox[0], bbox[1], bbox[2], bbox[3]

    #         closest_dist = np.inf
    #         closest_point = None
    #         bbox_center_x = (bbox_xmax + bbox_xmin)/2
    #         bbox_center_y = (bbox_ymax + bbox_ymin)/2

    #         for point in self.lidar_msg.points:
    #             point_world = np.array([[point.x],
    #                                     [point.y],
    #                                     [point.z],
    #                                     [1.0]])
                
    #             point_c = camera_matrix @ (rtlc @ point_world)
    #             pixel_x = point_c[0,0]//point_c[2,0]
    #             pixel_y = point_c[1,0]//point_c[2,0]
                
    #             # bbox 밖에 있으면 continue
    #             if(pixel_x < int(bbox_xmin) or pixel_x >= int(bbox_xmax) or pixel_y < int(bbox_ymin) or pixel_y > int(bbox_ymax) or np.isnan(pixel_x) or np.isnan(pixel_y)):
    #                 continue
                
    #             # dist = (bbox_center_x - pixel_x)**2 + (bbox_center_y - pixel_y)**2
    #             dist = abs(bbox_center_x - pixel_x) + abs(bbox_center_y - pixel_y) # 맨허튼 거리 공식
    #             if(dist < closest_dist):
    #                 closest_point = (point.x, point.y, point.z)
    #                 closest_dist = dist

    #             # bbox 안에 들어오는 모든 3D points에 대해 이미지에 표시
    #             img_visualize[int(pixel_y), int(pixel_x)] = (255,255,0)
            
    #         if(closest_point is not None):
    #             idx = rm.randint(0,99999)
    #             centroid_mrk_arry.markers.append(self.__make_marker__(closest_point, idx))
    #             print("bbox centroid point")
    #     return

    def get_rtlc(self, **parameters):
        alpha = parameters.get('alpha', 0)
        beta = parameters.get('beta', 0)
        tlc = parameters.get('tlc', 0)

        Rx = np.array([[1, 0, 0,0],
                        [0, np.cos(alpha), -np.sin(alpha),0],
                        [0, np.sin(alpha), np.cos(alpha),0],
                        [0,0,0,1]])

        Ryb = np.array([[np.cos(beta), 0, np.sin(beta),0],
                        [0, 1, 0,0],
                        [-np.sin(beta), 0, np.cos(beta),0],
                        [0,0,0,1]])

        Ry90 = np.array([[np.cos(np.deg2rad(-90)), 0, np.sin(np.deg2rad(-90)),0],
                         [0, 1, 0,0],
                         [-np.sin(np.deg2rad(-90)), 0, np.cos(np.deg2rad(-90)),0],
                         [0,0,0,1]])
                 
        Rx90= np.array([[1, 0, 0,0],
                        [0, np.cos(np.deg2rad(90)), -np.sin(np.deg2rad(90)),0],
                        [0, np.sin(np.deg2rad(90)), np.cos(np.deg2rad(90)),0],
                        [0,0,0,1]])
        
        T = np.array([[1, 0, 0, tlc[0]],      
                      [0, 1, 0, tlc[1]],                  
                      [0, 0, 1, tlc[2]],                       
                      [0, 0, 0, 1]]) 
        
        rtlc = Ryb@Rx@Ry90@Rx90@T
        return rtlc
    
    @staticmethod
    def __make_marker__(position , index):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.1)

        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]

        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w  = 0.0, 0.0, 0.0, 1.0

        marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.1, 0.1

        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 0.0, 1.0
        return marker
    
if __name__ == '__main__':
    try:
        sensor_fusion = SensorFusion()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

