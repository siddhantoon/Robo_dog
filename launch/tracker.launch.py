from launch import LaunchDescription
from launch_ros.actions import Node

# bringup->webcam -> yolotracker ->distance estimate->
def generate_launch_description():
	ld=LaunchDescription()
	
	webcam = Node(
		package ='btp_follower',
		executable='webcam'
	)
	distance = Node(
		package ='btp_follower',
		executable='distance_estimate'
	)
	# talker_node = Node(
	# 	package ='yolo_tracker_dp_sort',
	# 	executable='webcam'
	# )
	ld.add_action(webcam)
	ld.add_action(distance)

	return ld