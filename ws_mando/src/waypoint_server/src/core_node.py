#!/usr/bin/env python3

import rospy
from custom_msg_pkg.msg import WaypointArray  # 사용자 지정 메시지를 임포트합니다.
from std_msgs.msg import Int32  # std_msgs.msg 모듈에서 Int32 메시지를 가져옵니다.


class WaypointProcessor(object):

    def __init__(self):

        self.control_command = 0
        self.aeb = 0

        rospy.init_node('waypoint_processor_node', anonymous=True)
        self.waypoint_sub = rospy.Subscriber('waypoint_utm', WaypointArray, self.callback)
        
        # Publisher for the custom control message
        self.control_pub = rospy.Publisher('custom_control', Int32, queue_size=10)

        # Subscriber for get link addresses
        self.link_sub = rospy.Subscriber('current_link', Int32, self.link_callback)

        # Subscriber for emergency brake
        self.aeb_sub = rospy.Subscriber('AEB_switch', Int32, self.aeb_callback)

    def aeb_callback(self, aeb_msg):
        self.aeb = aeb_msg.data

    def link_callback(self, link_msg):
        link = link_msg.data

        #Estop 0
        #carcontrol 1
        #creep_carcontrol 2
        #ssp 3

        if link == 1:
            self.control_command = 1
            print("111")
        elif link == 2:
            if self.aeb == True:
                self.control_command = 0
                print("000")
            else:
                self.control_command = 2
                print("222")
        elif link == 3:
            self.control_command = 3
            print("333")
        else:
            self.control_command = 0
            print("444")

        # Create a custom control message and publish it
        control_msg = Int32()
        control_msg.data = self.control_command
        self.control_pub.publish(control_msg)
        print(link)

    def callback(self, waypoint_msg):
        # Handle waypoint data if needed
        pass

if __name__ == "__main__":
    try:
        waypoint_processor = WaypointProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
