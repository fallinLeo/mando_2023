#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import math

class LidarVisualization:
    def __init__(self):
        rospy.init_node('lidar_visualization_node')
        #유효 각도 범위 지정
        self.angle_min = -120.0 * np.pi / 180.0
        self.angle_max = 120.0 * np.pi / 180.0

        self.lidar_data = None
        self.marker_pub = rospy.Publisher('lidar_markers', Marker, queue_size=1)

        # 거리값을 발행할 퍼블리셔
        self.distance_pub = rospy.Publisher('lidar_distance', Float32, queue_size=10)

        rospy.Subscriber('scan', LaserScan, self.lidar_callback)

    def lidar_callback(self, data):
        self.lidar_data = data

    def visualize_lidar_data(self):
        if self.lidar_data is None:
            return

        
        ranges = np.array(self.lidar_data.ranges)
        ranges[np.isinf(ranges)] = 0  # 무한대 값을 0으로 대체
        ranges[ranges > 1e6] = 0      # 너무 큰 값도 0으로 대체

        #angles = np.linspace(self.lidar_data.angle_max, self.lidar_data.angle_min, len(ranges))
        angles = np.linspace(self.lidar_data.angle_min, self.lidar_data.angle_max, len(ranges))


        marker = Marker()
        marker.header.frame_id = self.lidar_data.header.frame_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # 점의 크기 조정
        marker.scale.y = 0.1
        marker.color.a = 1.0  # 알파 값 설정 (투명도)
        marker.color.r = 0.0  # 색상(빨강)
        marker.color.g = 1.0  # 색상(녹색)
        marker.color.b = 0.0  # 색상(파랑)
        marker.points = []  # 클러스터 중심점의 좌표를 추가할 리스트

        for i in range(len(ranges)):
            if not self.angle_min <= angles[i] <= self.angle_max:
                # 특정 각도 범위가 아닌 반대편에 있는 점들은 초록색으로 설정
                point = Point()
                point.x = ranges[i] * np.cos(angles[i])
                point.y = ranges[i] * np.sin(angles[i])
                point.z = 0.0  # Z 좌표는 0으로 설정 (평면 상에 표시)
                marker.points.append(point)

        self.marker_pub.publish(marker)

    def publish_distance(self):
        if self.lidar_data is not None:
        # Lidar scan 데이터가 유효한 경우
            
            ranges = np.array(self.lidar_data.ranges)
            ranges[np.isinf(ranges)] = 0  # 무한대 값을 0으로 대체
            ranges[ranges > 1e6] = 0      # 너무 큰 값도 0으로 대체
            angles = np.linspace(self.lidar_data.angle_min, self.lidar_data.angle_max, len(ranges))

            min_distance = float('inf')  # 무한대로 초기화하여 최소 거리를 찾기 위한 변수 설정

            for i in range(len(ranges)):
                if not self.angle_min <= angles[i] <= self.angle_max:
                    distance = self.lidar_data.ranges[i]
                    if not math.isinf(distance) and distance <min_distance:
                        min_distance = distance


            if not math.isinf(min_distance):
                # 유효한 최소 거리값을 찾았을 경우에만 발행
                distance_msg = Float32()
                distance_msg.data = min_distance
                self.distance_pub.publish(distance_msg)


    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.visualize_lidar_data()
            self.publish_distance()
            #rospy.loginfo("published data...")
            rate.sleep()

if __name__ == '__main__':
    try:
        lidar_visualization = LidarVisualization()
        lidar_visualization.run()
    except rospy.ROSInterruptException:
        pass
