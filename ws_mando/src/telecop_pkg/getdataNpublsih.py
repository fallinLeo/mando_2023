#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
# from std_msgs.msg import Int32
from geometry_msgs.msg import Quaternion, Pose, Point, Twist
from nav_msgs.msg import Odometry

steer_data = 0.0  # 초기값 설정
tele_drive = 0.0
encoder_val = 0.0

alpha = 0.2

# def lowpass_filter(data, alpha):
#     filtered_data = [data[0]]
#     for i in range(1, len(data)):
#         filtered_value = alpha * data[i] + (1 - alpha) * filtered_data[i - 1]
#         filtered_data.append(filtered_value)
#     return filtered_data

# def callback_teledrive(data):
#     global tele_drive
#     tele_drive = data.data

def callback_steer(data):
    global steer_data
    steer_data = data.data




def callback_drive(data):
    odom = Odometry()
    odom.header.frame_id = "encoder"  # Set the frame_id to "encoder"
    odom.header.stamp = rospy.Time.now()

    # Populate other fields of the Odometry message
    odom.pose.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist()

    # Set the linear velocity using the received drive data
    tele_drive = data.data
    encoder_val = tele_drive
    speed = encoder_val/34.89
    odom.twist.twist.linear.x = speed
    # odom.twist.twist.linear.x = data.data

    # filtered_data = lowpass_filter(data, alpha)

    
    # Set the angular velocity using the received steer data
    pot_value = steer_data
    # steer_angle = 0.0382 * pot_value -20.1832
    steer_angle = 0.0382 * pot_value + -21.1832
    odom.twist.twist.angular.z = steer_angle


    # Publish the Odometry message
    combined_odom_pub.publish(odom)






rospy.init_node('combine_and_publish', anonymous=True)

rospy.Subscriber("/drive_data_time_plot", Float32, callback_drive)
rospy.Subscriber("/steer_data_time_plot", Float32, callback_steer)

# 파이썬에서 publish하는 토픽 이름과 메시지 타입을 설정합니다.
combined_odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

rospy.spin()
