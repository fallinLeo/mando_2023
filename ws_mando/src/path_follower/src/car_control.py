#!/usr/bin/env python3

from os import path
import queue
import rospy
import math
import tf2_ros
import tf2_geometry_msgs
import numpy as np

from custom_msg_pkg.msg import WaypointArray, Waypoint
from geometry_msgs.msg import PointStamped, TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String, Float32, Int32
from nav_msgs.msg import Odometry



class CarControl(object):
    def __init__(self):
        self.waypoints_local = [] 

        self.mode_num = 0 #0 brake 1 normal 2 creep 3 ssp

        # target_speed 이전값
        self.target_steer_old = 0
        self.cte_old = 0

        self.location_available = False
        self.waypoints_available = False

        self.max_speed = 1.0    #1.8
        self.min_speed = 0.7        # 1.2
        self.scaling_factor = np.deg2rad(90) 

        self.sigma = 0

        # local_path subscribe
        self.local_path_sub = rospy.Subscriber('local_path', Path, self.callback_path)

        self.location_sub = rospy.Subscriber('odometry/filtered_map', Odometry, self.callback_control)

        #aeb subscrube
        self.mode_sub = rospy.Subscriber('custom_control',Int32,self.callback_mode)

        # target speed와 target steer publish
        self.control_pub = rospy.Publisher('teleop_cmd_vel', Twist, queue_size = 10)
        self.cte_error_pub = rospy.Publisher('cte_error', Float32, queue_size = 10)      

         
        pass
        
    def callback_mode(self,mode_data):
        self.mode_num = mode_data.data
        return

    def callback_path(self,local_path_msg):
        self.local_path_msg = local_path_msg
        return
    
    def callback_control(self,location_msg):
        if hasattr(self, 'local_path_msg') and hasattr(self, 'mode_num'):
            # 현재 속도 구하기
            vx = location_msg.twist.twist.linear.x
            vy = location_msg.twist.twist.linear.y
            current_speed = np.sqrt(vx**2 + vy**2)

            # 인하 성대 컨트롤 변수
            self.d_inha = current_speed*0.5    
            self.d_sung = 1 #self.present_speed*3 인식거리 4 > x > 3m 이상 

            # local path x,y,yaw값
            path_x, path_y, path_yaw = self.get_path_info(self.local_path_msg)

            
            #mode = {0 :'forward', 1 : 'back', 2 : 'stop' , 3 : 'slow'}
            mode = {0 :'stop', 1 : 'forward', 2 : 'slow' , 3 : 'back'}
            if(mode[self.mode_num] == 'forward'):
                target_steer, target_speed, cte = self.stanley_control(0,0,0, current_speed, path_x, path_y, path_yaw, L=0.5)
            # target_steer, target_speed, cte = self.inha_stanley_control(0,0,0, current_speed, path_x, path_y, path_yaw, L=0.5)

            elif(mode[self.mode_num] == 'slow'):
                target_steer, target_speed, cte = self.stanley_control(0,0,0, current_speed, path_x, path_y, path_yaw, L=0.5)
                target_speed = 0.7

            elif(mode[self.mode_num] == 'back'):
                target_steer, target_speed, cte = self.parking_control(0,0,0, current_speed, path_x, path_y, path_yaw, L=-0.25)
            
            else:
                target_steer, target_speed, cte = 530, 0, 0

            #steer 값을 최대 15도와 최소 -15도로 제한
            max_steer = 15.0  # 최대 허용 스티어 각도 (15도)
            min_steer = -15.0  # 최소 허용 스티어 각도 (-15도)
            target_steer = max(min_steer, min(max_steer, target_steer))
            target_steer = (target_steer + 21.1832) / 0.0382  #degree2pot_value


            print("목표 조향각 : ", target_steer)
            print("목표 속도값 : ", target_speed)
            print("#" * 30)
            CTE = Float32() # stanley 성능 비교 
            CTE.data = np.round(cte, 2) 
            self.cte_error_pub.publish(CTE)

            twist_msg = Twist()
            twist_msg.angular.z = target_steer
            twist_msg.linear.x = target_speed
            self.control_pub.publish(twist_msg)
        return
        
    def stanley_control(self, x, y, yaw, v, map_xs, map_ys, map_yaws, L) : 
        # ct error
        k = 0.5
        min_dist = 1e9
        min_dist_sung = 1e9

        min_index = 0
        min_index_sung = 0

        n_points = len(map_xs)

        front_x = x + L * np.cos(yaw)
        front_y = y + L * np.sin(yaw)
        
        # yaw_sung = yaw + self.d_sung * np.tan(self.sigma) / 1.06
        yaw_sung = yaw
        front_x_sung = front_x + self.d_sung * np.cos(yaw_sung) 
        front_y_sung = front_y + self.d_sung * np.sin(yaw_sung) 

        for i in range(n_points):
            dx = front_x - map_xs[i]
            dy = front_y - map_ys[i]

            dist = np.sqrt(dx * dx + dy * dy)
            if dist < min_dist:
                min_dist = dist
                min_index = i
            
            sung_dx = front_x_sung - map_xs[i]
            sung_dy = front_y_sung - map_ys[i]
            sung_dist = np.sqrt(sung_dx  * sung_dx  + sung_dy * sung_dy )
            if sung_dist < min_dist_sung :
                min_dist_sung  = sung_dist 
                min_index_sung = i   

        # compute cte at front axle
        map_x = map_xs[min_index]
        map_y = map_ys[min_index]
        map_yaw = map_yaws[min_index]
        dx = map_x - front_x
        dy = map_y - front_y
    
        perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)]
        cte = np.dot([dx, dy], perp_vec)

        # control law
        # print("현재 차량 yaw값 : ", math.degrees(yaw))
        # print("현재 map yaw값 : ", math.degrees(map_yaw))
        yaw_term = self.normalize_angle(map_yaw - yaw)
        cte_term = np.arctan2(k*cte, max(1.0, v))

        w_yaw = 0.5
        w_cte = 1

        # steering
        steer = w_yaw * yaw_term + w_cte * cte_term
        # print("cte_term : ", math.degrees(cte_term))
        # print("yaw term : ", math.degrees(yaw_term))
        # print("최종 steer : ", math.degrees(steer))
        sung_yaw_term = self.normalize_angle(map_yaws[min_index_sung] - yaw)

        speed = self.max_speed - abs(sung_yaw_term)/self.scaling_factor*(self.max_speed-self.min_speed)

        return -math.degrees(steer), speed, cte
    
    def inha_stanley_control(self, x, y, yaw, v, map_xs, map_ys, map_yaws, L) :
        k = 0.5
        # find nearest point
        # global k

        min_dist = 1e9
        min_dist_inha = 1e9
        min_dist_sung = 1e9

        min_index = 0
        min_index_inha = 0
        min_index_sung = 0

        n_points = len(map_xs) 
        front_x = x + L * np.cos(yaw)
        front_y = y + L * np.sin(yaw)

        yaw_inha = yaw + self.d_inha * np.tan(self.sigma) / 1.06
        front_x_inha = front_x + self.d_inha * np.cos(yaw_inha)     # front_x = front_x + self.d * np.cos(yaw) 
        front_y_inha = front_y + self.d_inha * np.sin(yaw_inha)     # front_y = front_y + self.d * np.sin(yaw) 

        yaw_sung = yaw + self.d_sung * np.tan(self.sigma) / 1.06
        front_x_sung = front_x + self.d_sung * np.cos(yaw_sung) 
        front_y_sung = front_y + self.d_sung * np.sin(yaw_sung) 

        for i in range(n_points) : # 
            dx = front_x - map_xs[i]
            dy = front_y - map_ys[i]

            dist = np.sqrt(dx * dx + dy * dy)
            if dist < min_dist:
                min_dist = dist
                min_index = i
            
            inha_dx = front_x_inha - map_xs[i]
            inha_dy = front_y_inha - map_ys[i]
            inha_dist = np.sqrt(inha_dx  * inha_dx  + inha_dy * inha_dy )
            if inha_dist < min_dist_inha :
                min_dist_inha  = inha_dist 
                min_index_inha = i   

            sung_dx = front_x_sung - map_xs[i]
            sung_dy = front_y_sung - map_ys[i]
            sung_dist = np.sqrt(sung_dx  * sung_dx  + sung_dy * sung_dy )
            if sung_dist < min_dist_sung :
                min_dist_sung  = sung_dist 
                min_index_sung = i   

        #print("stanley",min_index, min_index_inha)

        # compute cte at front axle
        map_x = map_xs[min_index]
        map_y = map_ys[min_index]
        #map_yaw = map_yaws[min_index]
        map_yaw = map_yaws[min_index_inha]
        dx = map_x - front_x
        dy = map_y - front_y

        perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)] # yaw값을 내적하기 위해서 임의로90도 돌려줌 오른쪽이 +방향 왼쪽방향이 -방향
        cte = np.dot([dx, dy], perp_vec) # 내적 해줌 Cross track error 

        # control law
        #	yaw_term = normalize_angle(map_yaw - yaw) * np.sin(np.pi/2 / (1+v/5))
        yaw_term = self.normalize_angle(map_yaw - yaw) #heading error
        cte_term = np.arctan2(k*cte, max(1.0, v)) # cross track error 의 yaw 값 받아옴. 
        w_yaw = 1
        w_cte = 1
        # steering
        steer = w_yaw * yaw_term + w_cte * cte_term
        # steer = -steer

        # print("HE : %f CTE :%f" %math.degrees(yaw_term) %math.degrees(cte_term))



        #CTE = Float32() # stanley 성능 비교 
        #CTE.data = np.round(cte, 2) 
        #self.pub_cte_error.publish(CTE) 
        #print("cte", cte)  max(self.scaling_factor, abs(sung_yaw_term))/
        sung_yaw_term = self.normalize_angle(map_yaws[min_index_sung] - yaw)

        speed = self.max_speed - abs(sung_yaw_term)/self.scaling_factor*(self.max_speed-self.min_speed)

        # deg_pub = Float32()
        # self.steer_deg_pub.data = deg_pub
        # self.steer_deg_pub.publish()
        return -math.degrees(steer), speed, cte #, [ w_yaw, w_cte, k, yaw_term, cte_term ]

    def parking_control(self, x, y, yaw, v, map_xs, map_ys, map_yaws, L) : 
        # ct error
        k = 0.5
        min_dist = 1e9

        min_index = 0

        n_points = len(map_xs)

        back_x = x + L * np.cos(yaw)
        back_y = y + L * np.sin(yaw)
    
        for i in range(n_points):
            dx = back_x - map_xs[i]
            dy = back_y - map_ys[i]
            dist = np.sqrt(dx * dx + dy * dy)
            if dist < min_dist:
                min_dist = dist
                min_index = i

        # compute cte at front axle
        map_x = map_xs[min_index]
        map_y = map_ys[min_index]
        map_yaw = map_yaws[min_index]
        dx = map_x - back_x
        dy = map_y - back_y
    
        perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)]
        cte = np.dot([dx, dy], perp_vec)

        # control law
        yaw_term = self.normalize_angle(map_yaw - yaw)
        cte_term = np.arctan2(k*cte, max(1.0, v))

        w_yaw = 0.5
        w_cte = 1

        # steering
        steer = w_yaw * yaw_term + w_cte * cte_term

        speed = -1.3

        return -math.degrees(steer), speed, cte

    def get_path_info(self, local_path_msg):
        path_x = []
        path_y = []
        path_yaw = []
        for pose_stamped in local_path_msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y

            # 쿼터니언에서 yaw 각도 구하기
            orientation = pose_stamped.pose.orientation
            yaw = self.euler_from_quaternion(orientation.x, orientation.y, orientation.z, orientation.w)

            path_x.append(x)
            path_y.append(y)
            path_yaw.append(yaw)
        return path_x, path_y, path_yaw
    
    @staticmethod
    def normalize_angle(angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle
    
    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return yaw_z  
    
    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
    
if __name__=='__main__':
    rospy.init_node('steer_control', anonymous=True)
    steer_control = CarControl()
    rospy.spin()