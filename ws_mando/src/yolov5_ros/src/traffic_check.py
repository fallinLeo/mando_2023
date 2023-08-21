
import rospy
from detection_msgs.msg import BoundingBox, BoundingBoxes
import os
import sys
from std_msgs.msg import String
from std_msgs.msg import Int32
from rostopic import get_topic_type

def bounding_box_callback(msg):
    # Initialize variables to keep track of most frequent class and its count
    class_count = {}
    most_frequent_class = None
    max_count =13
    checked_pub = rospy.Publisher('/frequent_check!',String,queue_size=10)
    filtered_pub = rospy.Publisher("/filtered_class", Int32, queue_size=10)
    
    # Iterate through bounding boxes in the message
    for bbox in msg.bounding_boxes:
        # Check if probability is greater than 70%
        if bbox.probability > 0.7:
            # Update class count
            if bbox.Class in class_count:
                class_count[bbox.Class] += 1
            else:
                class_count[bbox.Class] = 1
            
            # Update most frequent class and max count
            if class_count[bbox.Class] > max_count:
                max_count = class_count[bbox.Class]
                most_frequent_class = bbox.Class
                checked_pub.publish("checked")
    
    # Check if a most frequent class is found
    if most_frequent_class is not None:
        # Create a new BoundingBoxes message with only the most frequent class
        filtered_msg = Int32()
        for bbox in msg.bounding_boxes:
            if bbox.Class == most_frequent_class:
                filtered_msg.data = bbox.Class
        
        # Publish the filtered message
        filtered_pub.publish(filtered_msg)
    else :
        filtered_pub.publish(100)


if __name__ == "__main__":

    rospy.init_node("traffic_check")
       
    # BoundingBoxes 메시지 타입으로 topic 구독
    rospy.Subscriber("/yolov5_detections", BoundingBoxes, bounding_box_callback)
    
    rospy.spin()





