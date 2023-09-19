#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
import time
import numpy as np

from nav_msgs.msg import Odometry
from custom_msg_pkg.msg import WaypointArray, Waypoint
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathFollowingVisualization(object):
    def __init__(self):
        # 좌표 변환을 위한 TF2 Buffer와 TransformListener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transform = None

        # Closest Waypoint Array 메시지를 받을 subscriber
        self.waypoint_sub = rospy.Subscriber('close_waypoints', WaypointArray, self.callback_waypoints)

        # ekf를 통해 필터링한 현재 위치(frame : map) 받아오기
        self.location_sub = rospy.Subscriber('odometry/filtered_map', Odometry, self.callback_location)

        # Close Waypoints를 base_link frame으로 변환 후 publish
        self.close_waypoints_global_pub = rospy.Publisher('close_waypoints_global_visualization', MarkerArray, queue_size=10)
        self.close_waypoints_local_pub = rospy.Publisher('close_waypoints_local_visualization', MarkerArray, queue_size=10)

        # close_waypoints_local에 대해 Path 생성 후 publish
        self.local_path_pub = rospy.Publisher('local_path', Path, queue_size=10)

        # 현재 차량에 대한 visulization publish
        self.car_pub = rospy.Publisher('car_visualiztion', MarkerArray, queue_size=10)
        self.car_id = 0
        self.car_arrow_id = 0
        
        # 차량이 지나갔던 경로 publish
        self.path_record_pub = rospy.Publisher('path_record_visualization', Path, queue_size = 10)
        self.location_record = []

    def callback_waypoints(self, closest_waypoint_array):
        # 가장 가까운 웨이포인트만 저장하고 10Hz의 주기로 publish
        self.closest_waypoints = closest_waypoint_array
    
    def callback_location(self, location_msg):
        if not hasattr(self, 'transform'):
            return
        
        start_time = time.time()  # 함수 시작 시간 측정

        self.current_location = location_msg
        self.location_record.append((location_msg.pose.pose.position.x, location_msg.pose.pose.position.y))  # X와 Y 값을 튜플로 저장
        
        self.__delete_all_markers__()

        try:
            self.transform = self.tf_buffer.lookup_transform("base_link", "map", rospy.Time(0))
            if hasattr(self, 'closest_waypoints'):
                self.visualize_closest_waypoints()
    
            if hasattr(self, 'current_location'):
                self.visualize_current_location()
            
            if hasattr(self, 'location_record'):
                self.visualzie_location_record()

            if hasattr(self, 'local_points'):
                self.publish_local_path()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("현재 map -> base_link 변환 불가")
            return

        end_time = time.time()  # 함수 종료 시간 측정
        rospy.loginfo(f"update_transform_and_visualization processing time: {end_time - start_time:.4f} seconds")
        return
    
    def visualize_closest_waypoints(self):
        # visualize를 위한 Marker Array
        close_waypoints_global = MarkerArray()
        close_waypoints_local = MarkerArray()

        local_points = []

        for waypoint in self.closest_waypoints.waypoints:
            # Map frame에서의 close waypoints visualize
            m_map = self.__make_marker__(waypoint.x, waypoint.y, waypoint.index, "map")
            close_waypoints_global.markers.append(m_map)
            
            # Transform waypoint to base_link frame
            waypoint_map = PointStamped()
            waypoint_map.header.frame_id = "map"
            waypoint_map.point.x = waypoint.x
            waypoint_map.point.y = waypoint.y

            waypoint_base_link = tf2_geometry_msgs.do_transform_point(waypoint_map, self.transform)

            # base_link frame에서의 close waypoints visualize
            m_base_link = self.__make_marker__(waypoint_base_link.point.x, waypoint_base_link.point.y, waypoint.index, "base_link")
            close_waypoints_local.markers.append(m_base_link)
            local_points.append((waypoint_base_link.point.x, waypoint_base_link.point.y))

            
        # self.__delete_all_markers__()

        self.close_waypoints_global_pub.publish(close_waypoints_global)
        self.close_waypoints_local_pub.publish(close_waypoints_local)

        self.local_points = local_points
        return

    def visualize_current_location(self):
        # visualize를 위한 Marker Array
        car = MarkerArray()

        # map frame에서의 자동차 위치 및 방향 정보 추출
        map_x, map_y = self.current_location.pose.pose.position.x, self.current_location.pose.pose.position.y
        map_orientation = (
                        self.current_location.pose.pose.orientation.x,
                        self.current_location.pose.pose.orientation.y,
                        self.current_location.pose.pose.orientation.z,
                        self.current_location.pose.pose.orientation.w
                    )
        
        # map frame에서의 자동차 마커 생성 및 추가
        car_map = self.__make_car_marker__(map_x, map_y, map_orientation, "map")
        car.markers.append(car_map)
        arrow_map = self.__make_arrow_marker__(map_x, map_y, map_orientation, "map")
        car.markers.append(arrow_map)

        # Transform car position to base_link frame
        car_position_map = PointStamped()
        car_position_map.header.frame_id = "map"
        car_position_map.point.x = map_x
        car_position_map.point.y = map_y

        # car_position_base_link = tf2_geometry_msgs.do_transform_point(car_position_map, self.transform)

        # # base_link frame에서의 자동차 마커 생성 및 추가
        # car_base_link = self.__make_car_marker__(car_position_base_link.point.x, car_position_base_link.point.y , (0,0,0,1), "base_link")
        # car.markers.append(car_base_link)

        # # base_link frame에서의 화살표 마커 생성 및 추가
        # arrow_base_link = self.__make_arrow_marker__(car_position_base_link.point.x, car_position_base_link.point.y, (0,0,0,1), "base_link")
        # car.markers.append(arrow_base_link)

        # base_link frame에서의 자동차 마커 생성 및 추가
        car_base_link = self.__make_car_marker__(0, 0, (0,0,0,1), "base_link")
        car.markers.append(car_base_link)

        # base_link frame에서의 화살표 마커 생성 및 추가
        arrow_base_link = self.__make_arrow_marker__(0, 0, (0,0,0,1), "base_link")
        car.markers.append(arrow_base_link)

        # self.__delete_car_markers__()
        self.car_pub.publish(car)
        return
    
    def visualzie_location_record(self):
        # visualize를 위한 Path 메시지 생성
        path_msg = Path()
        path_msg.header.frame_id = "map"  # 경로의 좌표계를 설정 (이 경우 "map" 좌표계 사용)

        # location_record에 저장된 과거 위치 정보를 Path 메시지로 변환하여 추가
        for x, y in self.location_record:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0  # 자세 정보는 사용하지 않으므로 회전 없음

            path_msg.poses.append(pose)

        # Path 메시지를 publish
        self.path_record_pub.publish(path_msg)
        
    def publish_local_path(self):
        local_points = np.array(self.local_points)

        degree = 2
        coefficients = np.polyfit(local_points[:, 0], local_points[:, 1], degree)

        path_x = np.linspace(local_points[:, 0].min(), local_points[:, 0].max(), 50)
        path_y = np.polyval(coefficients, path_x)

        derivative_coefficients = np.polyder(coefficients)
        path_yaw = np.arctan(derivative_coefficients[0] + 2 * derivative_coefficients[1] * path_x)

        # Create Path message
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

        self.local_path_pub.publish(path_msg)   
        return
    
    def __make_marker__(self, x, y, index, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "closest_waypoint"
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.15)  # 0.1초 후에 Marker 삭제

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0

        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w  = 0.0, 0.0, 0.0, 1.0

        marker.scale.x, marker.scale.y, marker.scale.z = 0.3, 0.3, 0.3

        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.0, 0.0, 1.0
        return marker
    
    def __make_car_marker__(self, x, y, orientaion, frame_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "rectangle"
        marker.id = self.car_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.15)

        # 직사각형의 위치 설정
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0
        marker.pose.orientation.x = orientaion[0]
        marker.pose.orientation.y = orientaion[1]
        marker.pose.orientation.z = orientaion[2]
        marker.pose.orientation.w = orientaion[3]

        # 직사각형의 크기 설정
        marker.scale.x = 1.2  # 2m 길이
        marker.scale.y = 0.6  # 1m 폭
        marker.scale.z = 0.1  # 0.5m 높이

        # 직사각형의 색상 설정
        marker.color.a = 1.0
        marker.color.r = 255.0 / 255.0
        marker.color.g = 200.0 / 255.0
        marker.color.b = 255.0 / 255.0

        self.car_id = self.car_id + 1
        return marker

    def __make_arrow_marker__(self, x, y, orientation, frame_id="map"):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "arrow"
        marker.id = self.car_arrow_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        # marker.lifetime = rospy.Duration(0.2)
        
        # 화살표의 위치 및 방향 설정
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.11
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]

        # 화살표의 크기 설정
        marker.scale.x = 1.0  # 화살표의 길이
        marker.scale.y = 0.1  # 화살표의 폭
        marker.scale.z = 0.1  # 화살표의 높이

        # 화살표의 색상 설정
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0  # 파란색 화살표

        self.car_arrow_id = self.car_arrow_id + 1
        return marker

    def __delete_all_markers__(self):
        marker_array = MarkerArray()  # MarkerArray 생성
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)  # Marker를 MarkerArray에 추가
        self.car_pub.publish(marker_array)   # MarkerArray 발행

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
    
if __name__=='__main__':
    rospy.init_node('path_tracking', anonymous=True)
    path_following = PathFollowingVisualization()
    rospy.spin()
