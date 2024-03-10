#!/usr/bin/env python3

import queue
import rospy
import time
import math
import numpy as np
from scipy.spatial import Delaunay

from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped


class DelaunayPathPlanning():
    def __init__(self):
        rospy.init_node('delaunay_get_waypoints')
        self.obstacle_sub = rospy.Subscriber('/sensor_fusion_centroid_point', MarkerArray, self.obstacle_callback)
        # self.wall_pt_sub = rospy.Subscriber('/publish_line_markers', Marker, self.wall_pts_callback)
        self.wall_pt_sub = rospy.Subscriber('/wall_detection', MarkerArray, self.wall_pts_callback)

        self.waypoint_pub = rospy.Publisher('/waypoint', MarkerArray, queue_size = 10)
        self.visulize_pub = rospy.Publisher('/delaunay_points', MarkerArray, queue_size = 10)
        self.path_pub = rospy.Publisher('/path', Path, queue_size = 10)

        self.wall_points = []
        self.obstacle_points = []

        # 0.1초마다 callback 함수를 호출하여 데이터를 publish하는 코드
        self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_callback)

        return
    
    def timer_callback(self, event):
        first_time = time.time()

        filtered_points = [pt for pt in self.wall_points + self.obstacle_points if not np.isnan(pt).any()]
        waypoints = self.get_waypoints(filtered_points)
        if(len(waypoints) != 0 ):
                self.path_planning(waypoints)

        self.obstacle_points = []
        self.wall_points = []
        print("총 계산 시간 : ", time.time() - first_time)

        return
    
    def get_waypoints(self, points):
        print(len(points))
        if(len(points)<4):
            print("점이 네 개 미만면 들로네가 안돼요")
            return [[0,0], [1,0], [2,0], [3,0]]

        
        triangulation = Delaunay(points)
        points = np.array(points)

        triangles = points[triangulation.simplices]
        triangles_simplices = triangulation.simplices
        
        # 중심 구하기
        centers = points[triangulation.simplices].mean(axis=1)
        # 중심에서 (-3,0)을 빼기
        centers_shifted = centers - np.array([-5, 0])

        # (-3,0)에 대한 거리 계산
        centers_dist  = np.linalg.norm(centers_shifted, axis=1)

        # 가장 가까운 삼각형의 인덱스 추출
        closest_idx = np.argmin(centers_dist)
        closest_tri_simplicy = triangles_simplices[closest_idx]

        triangle_area_vectorized = np.vectorize(self.triangle_area, signature='(n,m)->()')
        areas = triangle_area_vectorized(triangles)
        
        # closest_tri_simplicy를 제외한 나머지 삼각형들
        # tris_temp = np.delete(triangles_simplices, closest_tri_simplicy, axis=1)
        # print("tris_temp : ",tris_temp)
        tris_temp = np.delete(triangles_simplices, closest_idx, axis=0)
        areas = np.delete(areas, closest_idx, axis=0)

        sorted_triangles_simplicies = []
        sorted_triangles_simplicies.append(closest_tri_simplicy)
        current_tri_simplicy = closest_tri_simplicy
        waypoints = []
        prev_tri_simplicy = None

        max_x_point_index = np.argmax(points[:, 0])  # x좌표가 가장 큰 점의 인덱스를 찾는다.

        for _ in range(len(tris_temp)):
            # 현재 삼각형과 두 점이 겹치는 삼각형을 찾는다.
            overlapping_points = np.sum(np.isin(tris_temp, current_tri_simplicy), axis=1)
            indices, = np.where(overlapping_points == 2)

            if len(indices) > 0:
                # 겹치는 점을 가진 삼각형 중에서 가장 길이가 긴 삼각형을 선택한다.
                lengths = []
                for idx in indices:
                    overlapping_indices = np.intersect1d(current_tri_simplicy, tris_temp[idx])
                    if len(overlapping_indices) == 2:
                        overlapping_points_coords = points[overlapping_indices]
                        length = np.linalg.norm(overlapping_points_coords[0] - overlapping_points_coords[1])
                        lengths.append(length)
                max_length_idx = np.argmax(lengths)
                current_idx = indices[max_length_idx]
                
                # 선택한 삼각형의 simplicy를 추출한다.
                current_tri_simplicy = tris_temp[current_idx]
                sorted_triangles_simplicies.append(current_tri_simplicy)

                # 이전 삼각형과 현재 삼각형이 겹치는 두 점을 찾는다.
                if prev_tri_simplicy is not None:
                    overlapping_indices = np.intersect1d(prev_tri_simplicy, current_tri_simplicy)

                    # 겹치는 두 점의 좌표를 가져와 중점을 계산한다.
                    # if len(overlapping_indices) == 2:
                    overlapping_points_coords = points[overlapping_indices]
                    midpoint = overlapping_points_coords.mean(axis=0)
                    waypoints.append(midpoint)

                # 처리한 삼각형은 목록에서 제거한다.
                # indices에 포함된 모든 인덱스를 삭제한다.
                tris_temp = np.delete(tris_temp, indices, axis=0)
                areas = np.delete(areas, indices, axis=0)

                prev_tri_simplicy = current_tri_simplicy

            else:
                break
            
            # x좌표가 가장 큰 점이 포함된 삼각형이면 for문을 탈출한다.
            if max_x_point_index in current_tri_simplicy:
                break

        # print(closest_tri_simplicy, sorted_triangles_simplicies)
        sorted_triangles = points[sorted_triangles_simplicies]
        sorted_triangles = sorted_triangles.reshape(-1, 2)
        # waypoints = waypoints[:15]
        waypoints_markers = self.__make_marker_array__(waypoints, (1,1,1))
        delaunay_points = self.__make_marker_array__(sorted_triangles, (0,0,1))

        self.waypoint_pub.publish(waypoints_markers)
        self.visulize_pub.publish(delaunay_points)
        return waypoints

    def triangle_area(self, triangle_points):
        """삼각형의 넓이를 계산하는 함수"""
        a = triangle_points[0]
        b = triangle_points[1]
        c = triangle_points[2]
        return 0.5 * abs((a[0]*(b[1]-c[1]) + b[0]*(c[1]-a[1]) + c[0]*(a[1]-b[1])))
    
    def path_planning(self, waypoints):
        # waypoints로부터 2차 회귀식 계산 후 publish

        waypoints.extend([[-3, 0], [-2, 0], [-1, 0]])
        # waypoints를 numpy 배열로 변환
        waypoints_np = np.array(waypoints)
        print("waypoints_np shape:", waypoints_np.shape, len(waypoints_np))

        degree = 3
        coefficients = np.polyfit(waypoints_np[:, 0], waypoints_np[:, 1], degree)

        path_x = np.linspace(waypoints_np[:, 0].min(), waypoints_np[:, 0].max(), 50)
        path_y = np.polyval(coefficients, path_x)

        derivative_coefficients = np.polyder(coefficients)
        path_yaw = np.arctan(derivative_coefficients[0] + 2 * derivative_coefficients[1] * path_x)

        # Path 메시지 생성
        path_msg = Path()
        path_msg.header.frame_id = "base_link"  

        for x, y, yaw in zip(path_x, path_y, path_yaw):
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose.position.x = x
            pose.pose.position.y = y
            quaternion = self.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            path_msg.poses.append(pose)

        
        # Path 메시지 퍼블리쉬
        self.path_pub.publish(path_msg)
        return path_msg
    
    def wall_pts_callback(self, wall_pts_msg):
        print("wall_pts_callback")
        self.wall_points = [(marker.pose.position.x, marker.pose.position.y) for marker in wall_pts_msg.markers]
        # #self.wall_points = [[wall_pt.x, wall_pt.y] for wall_pt in wall_pts_msg.points]
        # wall_start_pts = []
        # wall_end_pts   = []

        # for i, wall_pt in enumerate(wall_pts_msg.points):
        #     if(i%2==0): 
        #         # 시작점
        #         wall_start_pts.append((-wall_pt.x,-wall_pt.y))
        #     else:
        #         # 끝점
        #         wall_end_pts.append((-wall_pt.x,-wall_pt.y))

        # # self.wall_points = [(-wall_pt.x, -wall_pt.y) for wall_pt in wall_pts_msg.points]
        # self.wall_points = []
        # # 보간된 점 추가
        # for start_pt, end_pt in zip(wall_start_pts, wall_end_pts):
        #     if math.sqrt((start_pt[0]-end_pt[0])**2 + (start_pt[1]-end_pt[1])**2 )> 5:
        #         x_interp = np.linspace(start_pt[0], end_pt[0], num=10)  # 첫 번째와 마지막 점은 이미 포함되어 있으므로 제외
        #         y_interp = np.linspace(start_pt[1], end_pt[1], num=10)
        #         self.wall_points.extend(list(zip(x_interp, y_interp)))

        #     elif math.sqrt((start_pt[0]-end_pt[0])**2 + (start_pt[1]-end_pt[1])**2 )> 4:
        #         x_interp = np.linspace(start_pt[0], end_pt[0], num=8)  # 첫 번째와 마지막 점은 이미 포함되어 있으므로 제외
        #         y_interp = np.linspace(start_pt[1], end_pt[1], num=8)
        #         self.wall_points.extend(list(zip(x_interp, y_interp)))

        #     elif math.sqrt((start_pt[0]-end_pt[0])**2 + (start_pt[1]-end_pt[1])**2 )> 3:
        #         x_interp = np.linspace(start_pt[0], end_pt[0], num=6)  # 첫 번째와 마지막 점은 이미 포함되어 있으므로 제외
        #         y_interp = np.linspace(start_pt[1], end_pt[1], num=6)
        #         self.wall_points.extend(list(zip(x_interp, y_interp)))

        #     elif math.sqrt((start_pt[0]-end_pt[0])**2 + (start_pt[1]-end_pt[1])**2 )> 2:
        #         x_interp = np.linspace(start_pt[0], end_pt[0], num=4)  # 첫 번째와 마지막 점은 이미 포함되어 있으므로 제외
        #         y_interp = np.linspace(start_pt[1], end_pt[1], num=4)
        #         self.wall_points.extend(list(zip(x_interp, y_interp)))
        #     else:
        #         x_interp = np.linspace(start_pt[0], end_pt[0], num=2)  # 첫 번째와 마지막 점은 이미 포함되어 있으므로 제외
        #         y_interp = np.linspace(start_pt[1], end_pt[1], num=2)
        #         self.wall_points.extend(list(zip(x_interp, y_interp)))



        return
    
    def obstacle_callback(self, obstacle_msg):
        # obstacle_msg : MarkerArray
        print("obstacle_callback")
        obstacle_width = 0.5
        obstacle_height = 0.5
        self.obstacle_points = [(marker.pose.position.x, marker.pose.position.y) for marker in obstacle_msg.markers]
        
        return

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
            marker.scale.x = 0.1  # Adjust the size of the markers as needed
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            marker_array.markers.append(marker)
        return marker_array
    
    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
    
if __name__ == '__main__':
    try:
        delaunay = DelaunayPathPlanning()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass