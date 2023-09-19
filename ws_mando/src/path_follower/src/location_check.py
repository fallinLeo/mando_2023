#!/usr/bin/env python3

import rospy
import math
import heapq
import time

from nav_msgs.msg import Odometry
from custom_msg_pkg.msg import WaypointArray, Waypoint

from std_msgs.msg import Int32, Float32
from scipy.spatial import KDTree

class LocationCheck(object):
    def __init__(self, ):
        # 현재 링크
        self.current_link = 0

        # WayPoint(frame : map) 받아오기
        self.waypoint_sub = rospy.Subscriber('waypoint_map', WaypointArray, self.callback_waypoint)

        # ekf를 통해 필터링한 현재 위치(frame : map) 받아오기
        self.location_sub = rospy.Subscriber('odometry/filtered_map', Odometry, self.callback_location_check)

        # WayPoint와 현재 위치를 통해 Link publish
        self.link_pub = rospy.Publisher('current_link', Int32 , queue_size = 10)

        # 현재 링크에서의 진행도를 publish
        self.progress_percentage = rospy.Publisher('progress_percentage', Float32 , queue_size = 10)

        # Close Waypoints n개 publish
        self.close_waypoints_pub = rospy.Publisher('close_waypoints', WaypointArray, queue_size=10)

        # 몇 개의 close waypoints를 보내질에 대한 변수 
        self.num_closest_waypoints = 10

        # Waypoint 데이터를 저장할 리스트
        self.waypoints_data = {}

        # Waypoint 받은 여부에 대한 boolean형
        self.waypoint_available = False

        self.kdtree = None
        self.current_position = None
        
        pass

    def callback_waypoint(self, waypoint_array_msg):
        if not self.waypoint_available:
            for waypoint_msg in waypoint_array_msg.waypoints:
                waypoint_data = {
                    'x': waypoint_msg.x,
                    'y': waypoint_msg.y,
                    'index': waypoint_msg.index,
                }
                # Check if the link exists in the dictionary, if not, initialize it
                if waypoint_msg.link not in self.waypoints_data:
                    self.waypoints_data[waypoint_msg.link] = []
                self.waypoints_data[waypoint_msg.link].append(waypoint_data)

        if self.waypoints_data:
            self.waypoint_sub.unregister()
            self.waypoint_available = True
        return
    
    def callback_location_check(self, location_msg):
        start_time = time.time()  # 함수 시작 시간 측정

        # 현재 위치
        current_position = (location_msg.pose.pose.position.x, location_msg.pose.pose.position.y)

        # waypoint나 현재 위치가 활성화 안되어있으면 return
        if not self.waypoint_available or not current_position:
            print("Couldn't find any waypoints or current position not updated yet.")
            return

        # 현재 link 업데이트
        if self.current_link + 1 in self.waypoints_data:
            next_link_first_waypoint = self.waypoints_data[self.current_link + 1][0]
            dist_from_next_link = self.__get_distance__(current_position, (next_link_first_waypoint['x'], next_link_first_waypoint['y']))
            if dist_from_next_link < 1:
                self.current_link += 1
        else:
            rospy.logwarn("다음 링크의 데이터를 찾을 수 없습니다.")

        # KDtree를 이용해서 현재 위치로부터 가장 가까운 waypoints 10개 얻기
        waypoints_cur_link = [(wp['x'], wp['y']) for wp in self.waypoints_data[self.current_link]]
        self.waypoints_cur_link_kdtree = KDTree(waypoints_cur_link)
        _, indices = self.waypoints_cur_link_kdtree.query(current_position, self.num_closest_waypoints)
        closest_points = [self.waypoints_data[self.current_link][i] for i in indices]
        print(closest_points[0]['index'])

        # 가장 가까운 Waypoints publish
        self.close_waypoints_pub.publish([self.__make_waypoint_msg__(cp['x'], cp['y'], cp['index'], self.current_link) for cp in closest_points])

        # 현재 위치가 현재 링크에서 몇 % 도달했는지 계산
        first_index = self.waypoints_data[self.current_link][0]['index']
        last_index = self.waypoints_data[self.current_link][-1]['index']
        mid_index = closest_points[0]['index'] - first_index 

        # length 계산
        length = last_index - first_index
        progress_percentage = mid_index / length * 100
        progress_percentage_msg = Float32()
        progress_percentage_msg.data = progress_percentage
        self.progress_percentage.publish(progress_percentage_msg)

        link_msg = Int32()
        link_msg.data = self.current_link
        self.link_pub.publish(link_msg)

        end_time = time.time()
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

