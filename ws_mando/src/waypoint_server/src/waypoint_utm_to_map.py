#!/usr/bin/env python3

import rospy
import tf
from custom_msg_pkg.msg import WaypointArray, Waypoint
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

class UtmToMapTransformer:
    def __init__(self):
        # TF listener를 초기화한다.
        self.tf_listener = tf.TransformListener()

        # 0.1초 간격의 타이머를 추가
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_data)

        # WaypointArray 메시지를 받을 subscriber를 초기화한다.
        self.waypoint_sub = rospy.Subscriber('waypoint_utm', WaypointArray, self.callback_waypoints)

        # 변환된 WaypointArray 메시지를 발행할 publisher를 초기화한다.
        self.waypoint_pub = rospy.Publisher('waypoint_map', WaypointArray, queue_size=10)
        # rviz 상에서 visualize를 위한 MarkerArray 발행
        self.waypoint_visualize = rospy.Publisher('waypoint_for_visualization', MarkerArray, queue_size=10)

        # x,y,link,index 배열에 대한 Waypoint Array
        self.transformed_waypoint_array = WaypointArray()
        # visualize를 위한 Marker Array
        self.transformed_marker_array = MarkerArray()

        self.publish_available = False

    def callback_waypoints(self, waypoint_array_utm):
        print(len(waypoint_array_utm.waypoints))        
        if(self.publish_available == False):
            for waypoint in waypoint_array_utm.waypoints:
                # PointStamped 메시지를 사용하여 변환을 수행한다.
                utm_point = PointStamped()
                utm_point.header.frame_id = "utm"
                utm_point.point.x = waypoint.x
                utm_point.point.y = waypoint.y
                
                try:
                    # tf 발행까지 30초 기다리기
                    self.tf_listener.waitForTransform("map", "utm", rospy.Time.now(), rospy.Duration(30.0))
                    waypoint_map = self.tf_listener.transformPoint("map", utm_point)

                    # Waypoint
                    transformed_waypoint = Waypoint()
                    transformed_waypoint.x = waypoint_map.point.x
                    transformed_waypoint.y = waypoint_map.point.y
                    transformed_waypoint.index = waypoint.index
                    transformed_waypoint.link = waypoint.link

                    self.transformed_waypoint_array.waypoints.append(transformed_waypoint)

                    # Marker
                    self.transformed_marker_array.markers.append(self.__make_marker__(waypoint_map.point.x, waypoint_map.point.y, waypoint.index))

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr("TF Error: %s" % e)
            self.publish_available = True

    def publish_data(self, event):
        if self.publish_available:
            # Waypoint visualization을 위한 publish
            rospy.loginfo("Number of markers in transformed_marker_array: %d", len(self.transformed_marker_array.markers))
            self.waypoint_visualize.publish(self.transformed_marker_array)
            # Waypoint의 x,y,link,index를 publish
            self.waypoint_pub.publish(self.transformed_waypoint_array)

    @staticmethod
    def __make_marker__(x,y, index):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0

        marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w  = 0.0, 0.0, 0.0, 1.0

        marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.1, 0.1

        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 0.0, 1.0
        return marker
    
if __name__ == "__main__":
    rospy.init_node('utm_to_map_transform_node', anonymous=True)
    transformer = UtmToMapTransformer()
    rospy.spin()