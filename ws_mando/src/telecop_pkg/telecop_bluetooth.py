#!/usr/bin/env python

import rospy

if __name__ == "__main__":
    # Initialize the ROS node with a unique name 'telecop_node'
    rospy.init_node('telecop_node')

    # Your code here...

    # Keep the node alive until it is explicitly shutdown
    rospy.spin()
