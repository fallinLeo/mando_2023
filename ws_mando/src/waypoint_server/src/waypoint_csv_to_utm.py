#!/usr/bin/env python3

import rospy
import rospkg
import pandas as pd
from custom_msg_pkg.msg import Waypoint, WaypointArray

csv_file_name = "hitech_0921_final.csv"

class UtmConverter(object):
    def __init__(self, file=None):
        if file is None:
            raise ValueError("Trying to find the file, but it does not exist")
        
        self.waypoint_array = WaypointArray()

        old_link = None
        current_link_number = -1  # Initialize to -1 so that first link will be 0
        old_link_final_index = 0

        for index, row in csv_file.iterrows():
            data = row[0].split(',')
            x = float(data[2].strip())
            y = float(data[3].strip())
            link = int(data[0].strip())

            waypoint = Waypoint()
            waypoint.x = x
            waypoint.y = y

            if link != old_link:
                current_link_number += 1
                old_link = link
                old_link_final_index = index

            waypoint.link = current_link_number
            # waypoint.index = index - old_link_final_index
            waypoint.index = index
            self.waypoint_array.waypoints.append(waypoint)
            print(x, y)

if __name__ == "__main__":
	# "map_server" 패키지에 대한 주소(path)를 얻어오기 위한 코드
	rospack = rospkg.RosPack()
	path = rospack.get_path("waypoint_server")
	# 받아온 주소에서 csv파일에 대한 접근을 위한 주소를 더하기
	csv_file_path = path + "/csv_files/" + csv_file_name
	csv_file = pd.read_csv(csv_file_path, encoding= 'utf-8', sep = '|')

	# ROS
	rospy.init_node('waypoint_csv_to_utm_node', anonymous=True)
	# waypoint_visualize = rospy.Publisher('waypoint_visualize', MarkerArray, queue_size=10)
	waypoint_pub = rospy.Publisher('waypoint_utm', WaypointArray, queue_size=10)

	rate = rospy.Rate(10)  # 10 Hz

	utm_converter = UtmConverter(file = csv_file)
	
	while not rospy.is_shutdown():
		try:
			# waypoint_visualize.publish(utm_converter.marker_array)
			rospy.loginfo("Number of waypoints: %d", len(utm_converter.waypoint_array.waypoints))
			waypoint_pub.publish(utm_converter.waypoint_array)
			rate.sleep()
		except KeyboardInterrupt:
			pass

		
