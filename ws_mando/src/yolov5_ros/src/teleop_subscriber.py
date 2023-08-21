#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def cmd_vel_callback(msg):
    # 이 함수는 /teleop_cmd_vel 토픽에서 메시지가 도착할 때마다 호출됩니다.
    # 받은 메시지를 출력합니다.
    rospy.loginfo("Received linear velocity: x=%f, y=%f, z=%f",
                  msg.linear.x, msg.linear.y, msg.linear.z)
    rospy.loginfo("Received angular velocity: x=%f, y=%f, z=%f",
                  msg.angular.x, msg.angular.y, msg.angular.z)

def teleop_subscriber():
    # ROS 노드 초기화
    rospy.init_node('teleop_subscriber', anonymous=True)

    # /teleop_cmd_vel 토픽을 subscribe하고, 콜백 함수를 등록합니다.
    rospy.Subscriber("/teleop_cmd_vel", Twist, cmd_vel_callback)

    # ROS 스핀 (노드가 종료될 때까지 계속 실행됩니다.)
    rospy.spin()

if __name__ == '__main__':
    try:
        teleop_subscriber()
    except rospy.ROSInterruptException:
        pass
