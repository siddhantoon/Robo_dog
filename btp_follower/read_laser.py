import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,QoSDurabilityPolicy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserSubscriber(Node):
	def __init__(self):
		'''Takes a laser & Object detected[st-angle,end-angle,center-angle] return to the object.'''
		super().__init__("laser_subscriber")
		qos_profile = QoSProfile(
			durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )
		self.laser_subscription = self.create_subscription(
			LaserScan, "/scan", self.calc_dist,qos_profile=qos_profile
		)
		self.laser_subscription  # prevent unused variable warning

		self.laser_msg = None
		self.max_angle = math.radians(360) # we are considering 360 degrees as the max angle
		self.angles = []

	def calc_dist(self, msg):
		for angle in range(0,359,30):

			angle_index = self.angle_to_index(msg,angle)
			try:
				# get the distance at the specified angle
				distance = msg.ranges[angle_index]
				if distance<=msg.range_max and distance>=msg.range_min:
					distance = int(100*(distance))
				print(f"Angle:{angle} index:{angle_index} \nDistance:{distance}")
			except:
				print(f"Angle:{angle} index:{angle_index} Range:{len(msg.ranges)}")
				exit()
		# return distance

	def calc_dist_in_range(self,msg, st, en,process):
		'''
		Calculates list of distances between st & en angle and 
		processes using the function provided such as minimum or mean distance then returns it
		'''
		if msg == None:
			print('Laser not available')
			return None
		if st > en:
			range1_st, range1_en = self.angle_to_index(msg,0), self.angle_to_index(msg,st)
			range2_st, range2_en = self.angle_to_index(msg,en), self.angle_to_index(msg,359)
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
		angle_radians = math.radians(angle_degrees)

		return int((angle_radians/self.max_angle)*len(msg.ranges))
	
	def laser_scan_callback(self,msg):
		ranges = msg.ranges
		intensities = msg.intensities
		# Process the laser scan data as needed
		print("Received laser scan data")
		print("Number of ranges: ", len(ranges))
		print("Number of intensities: ", len(intensities))
		l = list(map(math.degrees,[msg.angle_min,msg.angle_max,msg.angle_increment]))
		m = 360
		ra = int((msg.angle_max -msg.angle_min)/msg.angle_increment)
		# print(l,msg.range_min,msg.range_max)
		print(ra,len(ranges))

    # for i in range(len(ranges)):
    #     print("Range: ", ranges[i])
    #     print("Intensity: ", intensities[i])
    #     print("")

def main(args=None):
	rclpy.init(args=args)
	laser_subscriber = LaserSubscriber()
	rclpy.spin(laser_subscriber)
	laser_subscriber.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()