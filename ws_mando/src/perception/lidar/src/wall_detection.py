#!/usr/bin/env python3
from turtle import left
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32, PointStamped
from visualization_msgs.msg import Marker, MarkerArray

from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline

import tf


class LaserROIFilter:
    def __init__(self):
        rospy.init_node('wall_detection', anonymous=True)

        self.laser_roi_filtered_sub = rospy.Subscriber('/laser_roi_filtered_for_wall_detection', PointCloud, self.callback_laser)
        self.wall_pub = rospy.Publisher('/wall_detection', MarkerArray, queue_size=10)

    def callback_laser(self, laser_msg):
        # 왼쪽과 오른쪽 점들을 나눠서 대입
        left_points = []
        right_points = []

        for point in laser_msg.points:
            if point.y > 0:
                left_points.append([point.x, point.y])
            else:
                right_points.append([point.x, point.y])


        left_wall_pts = self.detect_wall(left_points)
        right_wall_pts = self.detect_wall(right_points)

        print(left_wall_pts)
        wall_marker_arry = self.__make_marker_array__(left_wall_pts+right_wall_pts, (0,0,1))
        self.wall_pub.publish(wall_marker_arry)
        return
    
    def detect_wall(self, points):
        points = np.array(points)
        X = points[:,0].reshape(-1, 1) # reshape your data using array.reshape(-1, 1) if it contains a single feature
        Y = points[:,1]
        
        # RANSAC algorithm
        ransac = RANSACRegressor()
        ransac.fit(X, Y)
        inlier_mask = ransac.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)

        # Predict data of estimated models
        line_X = np.linspace(-2, 3, 4)[:, np.newaxis]
        line_y_ransac = ransac.predict(line_X)

       # Combine X and Y into a 2D array
        line_points = list(np.column_stack((line_X, line_y_ransac)))

        return line_points
    
    def __make_marker_array__(self, points, color):
        # Create a MarkerArray message
        marker_array = MarkerArray()

        # Define a marker for each point
        for i, (x, y) in enumerate(points):
            marker = Marker()
            marker.header.frame_id = "base_link"  # Replace "your_frame_id" with the appropriate frame ID
            marker.id = i
            marker.type = Marker.SPHERE
            marker.lifetime = rospy.Duration(0.1)
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.3  # Adjust the size of the markers as needed
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            marker_array.markers.append(marker)
        return marker_array
if __name__ == '__main__':
    roi_filter = LaserROIFilter()
    rospy.spin()

# def transform_laser_bl(self, pc_laser):
#     pc_bl = PointCloud()  # base_link 좌표계의 PointCloud로 변환된 데이터를 저장할 변수

#     try:
#         # laser 좌표계로부터 base_link 좌표계로의 변환 기다리기
#         self.tf_listener.waitForTransform("base_link", "laser", rospy.Time(0), rospy.Duration(3.0))

#         # 변환된 데이터를 저장할 변수 초기화
#         pc_bl.header = pc_laser.header
#         pc_bl.header.frame_id = "base_link"  # base_link 좌표계로 설정

#         # laser 좌표계로부터 base_link 좌표계로 변환된 데이터를 계산하여 pc_bl에 저장
#         for point in pc_laser.points:
#             point_stamped = PointStamped()
#             point_stamped.header = pc_laser.header
#             point_stamped.point = point

#             try:
#                 transformed_point = self.tf_listener.transformPoint("base_link", point_stamped)
#                 pc_bl.points.append(transformed_point.point)
#             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#                 rospy.logerr("TF Error: %s" % e)

#     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
#         rospy.logerr("TF Error: %s" % e)

#     return pc_bl