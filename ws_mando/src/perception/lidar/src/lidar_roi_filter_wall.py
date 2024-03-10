#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32, PointStamped

import tf

ANGLE_1 = -50.
ANGLE_2 = 50.

class LaserROIFilter:
    def __init__(self):
        rospy.init_node('lidar_roi_filter', anonymous=True)

        self.tf_listener = tf.TransformListener()
        
        # ROI 범위 각도로 지정
        self.angle_1 = np.radians(-180 - ANGLE_1)
        self.angle_2 = np.radians(+180 - ANGLE_2)

        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        self.laser_roi_filtered_pub = rospy.Publisher('/laser_roi_filtered_for_wall_detection', PointCloud, queue_size=10)

    def callback_laser(self, laser_msg):
        pointcloud_msg = self.filter_roi(laser_msg)
        # pointcloud_msg_bl = self.transform_laser_bl(pointcloud_msg)
        self.laser_roi_filtered_pub.publish(pointcloud_msg)

    def filter_roi(self, laser_msg):
        ranges = np.array(laser_msg.ranges)
        angles = np.linspace(laser_msg.angle_min, laser_msg.angle_max, len(ranges))

        # Filter out np.inf and nan values
        valid_indices = np.where(np.logical_and(np.isfinite(ranges), ~np.isnan(ranges)))[0]
        ranges = ranges[valid_indices]
        angles = angles[valid_indices]
        
        indices = np.where(np.logical_or(angles < self.angle_1, angles > self.angle_2))[0]
        
        try:
            # laser 좌표계로부터 base_link 좌표계로의 변환 기다리기
            self.tf_listener.waitForTransform("base_link", "laser", rospy.Time(0), rospy.Duration(3.0))

            # 변환된 데이터를 저장할 변수 초기화
            pc_bl = PointCloud()
            pc_bl.header.frame_id = "base_link"  # base_link 좌표계로 설정
            pc_bl.header.stamp = rospy.Time.now()

            for i in indices:
                p = Point32()
                p.x = ranges[i] * np.cos(angles[i])
                p.y = ranges[i] * np.sin(angles[i])
                p.z = 0.001

                point_stamped = PointStamped()
                point_stamped.header = laser_msg.header
                point_stamped.point = p

                try:
                    transformed_point = self.tf_listener.transformPoint("base_link", point_stamped)
                    pc_bl.points.append(transformed_point.point)
                    print("Trasnforming")
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr("TF Error: %s" % e)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Error: %s" % e)

        return pc_bl

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