#!/usr/bin/env python

import queue
import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from custom_msg_pkg.msg import WaypointArray, Waypoint
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, TransformStamped

class CarControl(object):
    def __init__(self):
        # 좌표 변환을 위한 TF2 Buffer와 TransformListener
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200))  # tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Closest Waypoint Array 메시지를 받을 subscriber
        self.waypoint_sub = rospy.Subscriber('closest_waypoints', WaypointArray, self.car_control_callback)

        # 조향각과 속도값 publish
        self.car_control_pub = rospy.Publisher('car_control', , queue_size = 10)
        pass

    def car_control_callback(self, waypoints):
        # waypoints(frame : map)을 waypoints(frame : base_link)로 옮긴 후 받아오기
        waypoints_local = self.convert_waypoints_map_to_base_link(waypoints)

        # waypoints 2차 함수 근사화

        # ct error 및 heading error 계산

        # 차량 제어 적용

        # 최종적으로 조향각과 속도 publish
        
            
    def convert_waypoints_map_to_base_link(self, waypoints):
        waypoints_local = []
        for waypoint in waypoints.waypoints:
            waypoint_map = PointStamped()
            waypoint_map.header.frame_id = "map"
            waypoint_map.point.x = waypoint.x
            waypoint_map.point.y = waypoint.y

            try:
                # Transform waypoint to base_link frame and add another marker
                transform = self.tf_buffer.lookup_transform("base_link", "map", rospy.Time(0))
                waypoint_base_link = tf2_geometry_msgs.do_transform_point(waypoint_map, transform)
                waypoints_local.append((waypoint_base_link.point.x, waypoint_base_link.point.y))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("TF2 Transformation not available yet!")
                return None
        return waypoints_local
    
if __name__=='__main__':
    rospy.init_node('car_control', anonymous=True)
    car_control = CarControl()
    rospy.spin()