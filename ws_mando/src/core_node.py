#!/usr/bin/env python3

from math import inf
import time
import rospy
from custom_msg_pkg.msg import WaypointArray  # 사용자 지정 메시지를 임포트합니다.
from std_msgs.msg import Int32,Bool,Float32  # std_msgs.msg 모듈에서 Int32 메시지를 가져옵니다.



class WaypointProcessor(object):

    def __init__(self):

        self.control_command = 0
        self.aeb = 0

        self.normal_driving_time = inf
        self.parking_time = inf
        self.traffic_sign_time = inf

        #lidar 변수
        self.obstracle_min_det = 1.5 #최소 장애물감지거리
        self.obstracle_check = False

        #traffic 변수
        self.traffic_mode = None

        rospy.init_node('waypoint_processor_node', anonymous=True)
        self.waypoint_sub = rospy.Subscriber('waypoint_utm', WaypointArray, self.callback)
        
        # Publisher for the custom control message
        self.control_pub = rospy.Publisher('custom_control', Int32, queue_size=10)

        # Subscriber for get link addresses
        self.link_sub = rospy.Subscriber('current_link', Int32, self.link_callback)

        # Subscriber for emergency brake
        self.aeb_sub = rospy.Subscriber('AEB_switch', Int32, self.aeb_callback)

        #Lidar_subscribe
        self.lidar_sub = rospy.Subscriber('lidar_distance', Float32, self.lid_callback)
        self.lidar_pub = rospy.Publisher('obstracle_check',Bool, queue_size=10)
        
        #Traffic_subscribe   0:green 1:left 2:red 3:straightleft 4: yellow
        self.traffic_sub = rospy.Subscriber('filtered_class', Int32, self.traffic_callback)

    def aeb_callback(self, aeb_msg):
        self.aeb = aeb_msg.data

    def link_callback(self, link_msg):
        link = link_msg.data

        #mode = {'forward' : 0, 'back' : 1, 'stop' : 2, 'slow' : 3}
        mode = {'stop' : 0, 'forward' : 0, 'slow' : 1,  'back' : 3}
        link = {'normal driving' : 0, 'parking' : 1, 'traffic sign' : 2}
        
        if link == ['normal driving']:
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

    def lid_callback(self,msg):
        distance = msg.data
        if distance <=self.obstracle_min_det :
            self.obstracle_check = True
            print('obstracle check!')
        else :
            self.obstracle_check = False
        lid_pub = Bool()
        lid_pub.data = self.obstracle_check
        self.lidar_pub.publish(lid_pub)     
    
    def traffic_callback(self,msg):
        self.traffic_mode = msg.data
        print('traffic sign : ' , self.traffic_mode)
        pass


if __name__ == "__main__":
    try:
        waypoint_processor = WaypointProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
