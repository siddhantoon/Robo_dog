import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,QoSDurabilityPolicy
import datetime

class FollowPoint(Node):
	def __init__(self):
		super().__init__('follow_point')
		qos_profile = QoSProfile(
			depth=10
		)
		self.publisher_ = self.create_publisher(Twist, '/cmd_vel', qos_profile)

		self.subscription = self.create_subscription(
			Float32MultiArray,
			'/dist_angle',
			self.listener_callback,
			qos_profile
		)
		self.current_twist = Twist()
		self.linear = 0.02
		self.angular = 1.5
		self.update_time = datetime.datetime.now()
		# self.msg = Float32MultiArray(data=[])

	def listener_callback(self, msg):
		# t = abs(datetime.datetime.now() - self.update_time).total_seconds()*1000
		self.update_time = datetime.datetime.now()
		# print(f'After {t} ms')
		if len(msg.data)!=0:
			
			distance, angle = msg.data
			# distance in meters angle in degrees
			angle_radians = math.radians(-angle)

			distance = int(1000*distance)
			if distance>500:
				self.linear = 0.03
				self.angular = 3
				if distance>900:
					self.linear = 0.04

				linear_speed = math.log(distance)

				linear_vel = linear_speed*self.linear # Change the factor as needed
				angular_vel = angle_radians *self.angular # Change the factor as needed
				# if abs(angular_vel)<0.03:
				# 	angular_vel = 0.0
				# if angular_vel>0.3:
				# 	angular_vel = 0.3
				# if linear_vel>0.3:
				# 	linear_vel = 0.3
				# Update the current twist
				self.current_twist.linear.x = linear_vel
				self.current_twist.angular.z = angular_vel
				# Publish the current twist to cmd_vel topic
				self.publisher_.publish(self.current_twist)
				print(f'Distance: {distance} Angle: {angle} \nMoving:{self.current_twist.linear.x,self.current_twist.angular.z}')
			else:
				print('Stop')
				self.publisher_.publish(Twist())
		else:
			# pass
			print('Stop')
			self.publisher_.publish(Twist())


def main(args=None):
	rclpy.init(args=args)
	follow_point = FollowPoint()
	rclpy.spin(follow_point)
	follow_point.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
