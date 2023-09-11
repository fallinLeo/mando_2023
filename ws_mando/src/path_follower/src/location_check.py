#!/usr/bin/env python3

import rospy
import math
import heapq
import time

from nav_msgs.msg import Odometry
from custom_msg_pkg.msg import WaypointArray, Waypoint

from std_msgs.msg import Int32
from scipy.spatial import KDTree

class LocationCheck(object):
    def __init__(self, ):
        # WayPoint(frame : map) 받아오기
        self.waypoint_sub = rospy.Subscriber('waypoint_map', WaypointArray, self.callback_waypoint)

        # ekf를 통해 필터링한 현재 위치(frame : map) 받아오기
        self.location_sub = rospy.Subscriber('odometry/filtered_map', Odometry, self.callback_location_check)

        # WayPoint와 현재 위치를 통해 Link publish
        self.link_pub = rospy.Publisher('current_link', Int32 , queue_size = 10)

        # Close Waypoints n개 publish
        self.close_waypoints_pub = rospy.Publisher('close_waypoints', WaypointArray, queue_size=10)

        # 몇 개의 close waypoints를 보내질에 대한 변수 
        self.num_closest_waypoints = 10

        # Waypoint 데이터를 저장할 리스트
        self.waypoints_data = []

        # Waypoint 받은 여부에 대한 boolean형
        self.waypoint_available = False

        self.kdtree = None
        self.current_position = None
        
        pass

    def callback_waypoint(self, waypoint_array_msg):
        if(self.waypoint_available == False):
            for waypoint_msg in waypoint_array_msg.waypoints:
                waypoint_data = {
                    'x' : waypoint_msg.x,
                    'y' : waypoint_msg.y,
                    'index' : waypoint_msg.index,
                    'link' : waypoint_msg.link
                }
                self.waypoints_data.append(waypoint_data)
                # Create KD-Tree after receiving waypoints
            waypoints_coords = [(wp['x'], wp['y']) for wp in self.waypoints_data]
            self.kdtree = KDTree(waypoints_coords)

        if(len(self.waypoints_data) != 0):
            # 데이터를 받은 후에 subscriber를 종료 (데이터를 한 번만 받기 위해서)
            self.waypoint_sub.unregister()
            self.waypoint_available = True
        pass

    def callback_location_check(self, location_msg):
        start_time = time.time()  # 함수 시작 시간 측정

        self.current_position = (location_msg.pose.pose.position.x, location_msg.pose.pose.position.y)
        if not self.waypoint_available or not self.current_position:
            print("Couldn't find any waypoints or current position not updated yet.")
            return

        # KD Tree를 활용하여 현재 위치로부터 제일 가까운 waypoints n개 탐지
        _, indices = self.kdtree.query(self.current_position, self.num_closest_waypoints)
        closest_points = [self.waypoints_data[i] for i in indices]
        
        # 현재 링크 확인
        if hasattr(self, 'current_link'):
            current_link = self.current_link
        else:
            current_link = closest_points[0]['link']

        # 현재 링크와 다른 링크의 웨이포인트 수 계산
        different_link_points = [point for point in closest_points if point['link'] != current_link]

        # 현재 링크와 다른 링크의 웨이포인트가 2개 이상일 경우 링크 업데이트
        if len(different_link_points) >= 2:
            self.current_link = different_link_points[0]['link']

        # 가장 가까운 Waypoints publish
        self.close_waypoints_pub.publish([self.__make_waypoint_msg__(cp['x'], cp['y'], cp['index'], cp['link']) for cp in closest_points])

        link_msg = Int32()
        link_msg.data = current_link
        self.link_pub.publish(link_msg)
        
        end_time = time.time()  # 함수 종료 시간 측정
        rospy.loginfo(f"processing time: {end_time - start_time:.4f} seconds")
        return
        
    @staticmethod
    def __get_distance__(point1, point2):
        return (point1[0] - point2[0])**2 + (point1[1] - point2[1])**2
    
    @staticmethod
    def __make_waypoint_msg__(x,y,index,link):
        waypoint = Waypoint()
        waypoint.x = x
        waypoint.y = y
        waypoint.index = index
        waypoint.link = link
        return waypoint

if __name__=='__main__':
    rospy.init_node('location_checker', anonymous=True)
    location_check = LocationCheck()
    rospy.spin()

