#!/usr/bin/env python3

from math import inf
import time
import rospy
from custom_msg_pkg.msg import WaypointArray  # 사용자 지정 메시지를 임포트합니다.
from std_msgs.msg import Int32,String  # std_msgs.msg 모듈에서 Int32 메시지를 가져옵니다.


class WaypointProcessor(object):

    def __init__(self):

        self.control_command = 0
        self.aeb = 0

        self.normal_driving_time = inf
        self.parking_time = inf
        self.traffic_sign_time = inf

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

        #mode = {'forward' : 0, 'back' : 1, 'stop' : 2, 'slow' : 3}
        mode = {'stop' : 0, 'forward' : 0, 'slow' : 1,  'back' : 3}
        link = {'normal driving' : 0, 'parking' : 1, 'traffic sign' : 2}
        
        if link == ['normail driving']:
            self.normal_driving_time = time.time()
            self.control_command = mode['forward']
            print("forward mode")

        elif link == ['parking']:
            self.parking_time = time.time()
            if(self.parking_time - self.normal_driving_time >= 1):
                if self.aeb == True:
                    self.control_command = mode['stop']
                    print("stop mode")
                else:
                    self.control_command = mode['back']
                    print("back mode")
            else:
                self.control_command = mode['stop']
                print("stop mode")
                
        elif link == 2:
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
