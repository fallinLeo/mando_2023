#!/usr/bin/env python3

import queue
import rospy
import numpy as np
import cv2
import glob
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

CAMERA_HEIGHT = 360
CAMERA_WIDTH = 1280

class CombineCamera():
    def __init__(self):
        rospy.init_node('combine_camera')

        self.left_sub = rospy.Subscriber("/image_left",Image, self.callback_left_img)
        # self.left_sub = rospy.Subscriber("/image_example/raw",Image, self.callback_left_img)
        self.right_sub = rospy.Subscriber("/image_right",Image, self.callback_right_img)

        self.combined_pub = rospy.Publisher("/image_combined", Image, queue_size = 10)

        self.camera_window = np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8)
        return
        
    def callback_left_img(self, left):
        bridge = CvBridge()
        left_img = bridge.imgmsg_to_cv2(left, desired_encoding='bgr8')

        self.camera_window[:, :640] = left_img

        image_message = bridge.cv2_to_imgmsg(self.camera_window, encoding="bgr8")
        image_message.header.stamp = rospy.Time.now()
        self.combined_pub.publish(image_message)
        return

    def callback_right_img(self, right):
        bridge = CvBridge()
        right_img = bridge.imgmsg_to_cv2(right, desired_encoding='bgr8')

        self.camera_window[:, 640:] = right_img

        image_message = bridge.cv2_to_imgmsg(self.camera_window, encoding="bgr8")
        image_message.header.stamp = rospy.Time.now()
        # self.combined_pub.publish(image_message)
        return
    
if __name__ == '__main__':
    try:
        combine_cam = CombineCamera()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass