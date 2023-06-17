import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MinimalPublisher(Node):

	def __init__(self):
		super().__init__('minimal_publisher')
		self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
		timer_period = 0.05  # seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0

	def timer_callback(self):
		try:
			self.move_for(0.1,1)
			self.rotate(0.1,1)
			self.move_for(0.1,1)
			self.rotate(0.1,1)
			self.move_for(0.1,1)
			self.rotate(0.1,1)
			self.move_for(0.1,1)
			self.rotate(0.1,1)
			exit()
		except:
			self.publisher_.publish(Twist())

	def move_for(self,speed,duration):
		twist = Twist()
		twist.linear.x = speed
		self.publisher_.publish(twist)
		time.sleep(duration)
		twist.linear.x=0.0
		self.publisher_.publish(twist)
	def rotate(self,speed,duration):
		twist = Twist()
		twist.angular.z = speed
		self.publisher_.publish(twist)
		time.sleep(duration)
		twist.angular.z = 0.0
		self.publisher_.publish(twist)

def main(args=None):
	rclpy.init(args=args)

	minimal_publisher = MinimalPublisher()

	rclpy.spin(minimal_publisher)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()