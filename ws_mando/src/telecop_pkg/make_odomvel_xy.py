import rospy
import math
import numpy as np
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

class Calculate_odometry:
    def __init__(self):
        rospy.init_node('odometry_xy')
        

        rospy.Subscriber('odom_xvel',int,self.callback)
        rospy.Subscriber('steer_yvel',int,self.steerback)
        # pub = Int32MultiArray()

        # rospy.Publisher('output_odom_vel')
        self.rate = rospy.Rate(10)

    def callback(self,data):
        input_xvel = data.data

    def steerback(self,data):
        global angle_rad
        angle_rad+=data.data

    def rotation_matrix_3d(axis, angle_deg):
        # Args:
        # - axis: 회전 축을 나타내는 문자열 ('x', 'y', 또는 'z')
        # - angle_deg: 회전 각도 (도수법) 

        # deg to rad
        angle_rad = np.radians(angle_deg)

        # 회전 축에 따라 회전 행렬 생성
        if axis == 'x':
            rotation_matrix = np.array([
                [1, 0, 0],
                [0, np.cos(angle_rad), -np.sin(angle_rad)],
                [0, np.sin(angle_rad), np.cos(angle_rad)]
            ])
        elif axis == 'y':
            rotation_matrix = np.array([
                [np.cos(angle_rad), 0, np.sin(angle_rad)],
                [0, 1, 0],
                [-np.sin(angle_rad), 0, np.cos(angle_rad)]
            ])
        elif axis == 'z':
            rotation_matrix = np.array([
                [np.cos(angle_rad), -np.sin(angle_rad), 0],
                [np.sin(angle_rad), np.cos(angle_rad), 0],
                [0, 0, 1]
            ])
        else:
            raise ValueError("input valid rotate axis. ('x', 'y', or 'z')")

        return rotation_matrix


    
    def run(self):
        while not rospy.is_shutdown():

            


            self.rate.sleep()






if __name__ == '__main__':
    try:
        my_node = Calculate_odometry()
        my_node.run()
    except rospy.ROSInterruptException:
        pass