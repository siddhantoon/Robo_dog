import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from threading import Thread
class LaserSubscriber(Node):
	def __init__(self):
		super().__init__("laser_subscriber")
		qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
		qos_profile2 = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
		self.laser_subscription = self.create_subscription(
			LaserScan, "/scan", self.laser_callback,qos_profile=qos_profile
		)
		self.laser_subscription  # prevent unused variable warning
		self.tracker_subscription = self.create_subscription(
			Int32MultiArray, "/tracked_angle", self.tracker_callback, qos_profile=qos_profile2
		)
		self.tracker_subscription
		self.zero_angle = 0
		self.last_angle = 359
		self.laser_msg = None
		self.angles = []

	def calc_dist(self, msg, angle):

		angle = math.radians(angle)
		# get the angle index
		angle_index = int((msg.angle_max - angle) / msg.angle_increment)

		# get the distance at the specified angle
		distance = msg.ranges[angle_index]
		return distance

	def calc_dist_in_range(self,msg, st, en,process):
		'''
		Calculates list of distances between st & en angle and 
		processes using the function provided such as minimum or mean distance then returns it
		'''
		if msg == None:
			print('Laser not available')
			return None
		if st > en:
			range1_st, range1_en = self.angle_to_index(msg,0), self.angle_to_index(msg,en)
			range2_st, range2_en = self.angle_to_index(msg,st), self.angle_to_index(msg,359)
			distances = msg.ranges[range1_st:range1_en] + msg.ranges[range2_st:range2_en]
		else:
			distances = msg.ranges[
				self.angle_to_index(msg,st) : self.angle_to_index(msg,en)
			]
		# print(distances)
		distances = [i for i in distances if (i!= float('inf')) and(i!= float('nan'))]
		if len(distances) == 0:
			return None
		distance = process(distances)
		return distance

	def angle_to_index(self,msg, angle_degrees):
		angle_min = msg.angle_min
		angle_max = msg.angle_max
		angle_increment = msg.angle_increment
		angle_radians = math.radians(angle_degrees)

		return int((angle_radians - angle_min) / angle_increment)

	def laser_callback(self, msg):
		# Updating the laser_scan
		print('laser updating')
		self.laser_msg = msg
		msg = self.laser_msg
		angles = self.angles
		if len(angles):
			print(f'Detecting Human at angle of {angles[2]} degrees from centre of Camera.')
			if len(angles):
				distance = self.calc_dist_in_range(msg, angles[0], angles[1], min)
				if distance != None:
					central_angle = angles[2]
					print(distance, )
					print(f'Detecting Human at angle of {central_angle} degrees from centre of Camera.\nDistance:{distance}')
		
	def tracker_callback(self, angles):
		print('in tracker callback')
		angles = list(angles.data)
		self.angles = angles
		# msg = self.laser_msg
		# if len(angles):
		# 	print(f'Detecting Human at angle of {angles[2]} degrees from centre of Camera.')
		# 	if len(angles):
		# 		distance = self.calc_dist_in_range(msg, angles[0], angles[1], min)
		# 		if distance != None:
		# 			central_angle = angles[2]
		# 			print(distance, central_angle)
				# take the position of robot
				# calculate position of target wrt to robot using central angle and distance
				# publish the position to the Nav2 goal


def main(args=None):
	rclpy.init(args=args)
	laser_subscriber = LaserSubscriber()
	rclpy.spin(laser_subscriber)
	laser_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()


# angles = {0: [350, 10], 90: [80, 100]}
# for angle, sten in angles.items():
# 	distances = self.calc_dist_in_range(sten[0], sten[1])
# 	# print the distance
# 	std = np.std(distances)
# 	mean = np.mean(distances)
# 	minimum = min(distances)
# 	maximum = max(distances)
# 	print(f"xxxx\nAngle: {angle} \nmin: {minimum} \nmaximum: {maximum} \nstd: {std} \nmean: {mean}\nxxxx")
