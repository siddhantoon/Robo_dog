import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray,Float32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,QoSDurabilityPolicy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from threading import Thread
import datetime
class LaserSubscriber(Node):
	def __init__(self):
		'''Takes a laser & Object detected[st-angle,end-angle,center-angle] return to the object.'''
		super().__init__("distance_estimation")
		qos_profile = QoSProfile(
			durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
		self.laser_subscription = self.create_subscription(
			LaserScan, "/scan", self.laser_callback,qos_profile=qos_profile
		)
		self.laser_subscription  # prevent unused variable warning
		self.tracker_subscription = self.create_subscription(
			Int32MultiArray, "/detections", self.tracker_callback, qos_profile=qos_profile
		)
		self.tracker_subscription
	
		self.publisher = self.create_publisher(Float32MultiArray, "/dist_angle", 10)

		self.laser_msg = None
		self.max_angle = math.radians(359.9) # we are considering 360 degrees as the max angle
		self.detections = []
		self.camera_fov = 40
		self.tracker_update_time = datetime.datetime.now()
		self.laser_update_time = datetime.datetime.now()

	def calc_dist(self, msg, angle):

		angle = math.radians(angle)
		# get the angle index
		angle_index = int((msg.angle_max - angle) / msg.angle_increment)

		# get the distance at the specified angle
		distance = msg.ranges[angle_index]
		return distance
	
	def calc_angle_distance_from_detections(self,laser_msg,detections,process):
		'''Takes x1,x2 image width and returns the distance for Bbox and central angle in degrees, considering the camera FOV as 60 degrees
		applies function process on multiple distances. Returns distance in metres'''
		x1,x2,img_width = detections
		mid = img_width//2
		central_x = (x2+x1)//2
		to_degree = self.camera_fov/img_width
		# print(f'Mid value: {mid} central_x: {central_x}\nx1: {x1} x2: {x2}')
		central_angle = (central_x-mid)*to_degree
		if ((x2-x1)/img_width) > 0.9:
			return None, central_angle

		if (x1 <= mid) & (x2>=mid):
			# print('In Centre')
			rng1 = [0,(mid-x1)*to_degree]
			rng2 = [360- (x2-mid)*to_degree , 359.9]
			range1_st, range1_en = [self.angle_to_index(laser_msg,i) for i in rng1]
			range2_st, range2_en = [self.angle_to_index(laser_msg,i) for i in rng2]
			distances = laser_msg.ranges[range1_st:range1_en] + laser_msg.ranges[range2_st:range2_en]
		
		if (x1 < mid) & (x2<=mid):
			# print('On Left')
			st_angle = (mid-x2)*to_degree
			en_angle = (mid-x1)*to_degree
			distances = laser_msg.ranges[self.angle_to_index(laser_msg,st_angle) : self.angle_to_index(laser_msg,en_angle)]
		if (x1>= mid) & (x2>mid):
			# print('On Right')
			st_angle = 360- (x2-mid)*to_degree
			en_angle = 360- (x1-mid)*to_degree
			distances = laser_msg.ranges[self.angle_to_index(laser_msg,st_angle) : self.angle_to_index(laser_msg,en_angle)]
		distances = [i for i in distances if (i>=laser_msg.range_min) & (i<=laser_msg.range_max)]
		if len(distances):
			return process(distances), central_angle
		else:
			return None, central_angle
		
	def angle_to_index(self,msg, angle_degrees):
		angle_radians = math.radians(angle_degrees)
		return int((angle_radians/self.max_angle)*len(msg.ranges))

	def laser_callback(self, msg):
		'''Calculates the distance of human detected by the angle saved in state'''
		# print('laser updating')
		self.laser_update_time = datetime.datetime.now()
		# Updating the laser_scan
		self.laser_msg = msg
		if (abs(self.laser_update_time - self.tracker_update_time).total_seconds()*1000)>=100:
			self.detections = []
		detections = self.detections
		if len(detections):
			# Will calculate & publish the distance only if tracking was done within 100 milliseconds
			distance, central_angle = self.calc_angle_distance_from_detections(msg, detections, minavg)
			if distance != None:
				print(f'Detecting Human at angle of {central_angle} degrees from centre of Camera.\nDistance:{int(100*distance)}')
				dist_angle = Float32MultiArray(data=[distance,central_angle])
				self.publisher.publish(dist_angle)
		else:
			print(f'No Human detected')
			self.publisher.publish(Float32MultiArray(data=[]))

	def tracker_callback(self, detections):
		'''Updates the angles(by B-Boxes from the tracker)'''
		self.tracker_update_time = datetime.datetime.now()
		# print('In tracker callback')
		detections = list(detections.data)
		self.detections = detections
def minavg(l):
	# print('xxxxxxxxxxxxxxxxx')
	# print(l)
	if len(l)!=0:
		tot = len(l)
		l.sort()
		l = l[:int(0.5*tot)]
		# print('New l',l)
		if len(l)==0:
			return None
		else:
			return np.mean(l)
	return 0
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
