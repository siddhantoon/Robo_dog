import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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

		self.dist_angle_sub = self.create_subscription(
			Float32MultiArray,
			'/dist_angle',
			self.da_callback,
			qos_profile
		)
		self.dist_angle_sub
		self.dist_angle = []

		self.odom_sub = self.create_subscription(
			Odometry,
			'/odom',
			self.odom_callback,
			10)
		self.odom_sub
		self.odom_msg = Odometry()

		self.current_twist = Twist()

		# dist_angle update counter
		self.da_update = datetime.datetime.now()
		self.da_counter = 0
		
		timer_period = 0.1# seconds
		self.timer = self.create_timer(timer_period, self.control_callback)
		
	# dist angle store 
	# odom msg store
	# calc x-d,y-d 
	# calc x_ddot, y_ddot
	# run the control
	# check edge cases
	

	def odom_callback(self, msg):
		self.msg = msg

	def da_callback(self, msg):
		self.dist_angle = msg.data
		self.da_update = datetime.datetime.now()
		self.da_counter = 
	
	def control_callback(self):
		'''Publishes control commands '''
		msg = self.msg
		print(f'Actual:\n theta:{msg.pose.pose.orientation.z}\n V: {msg.twist.twist.linear.x}\n omega: {msg.twist.twist.angular.z}\n')
		twist = Twist()
		deltaT = (datetime.now() - self.prev_time).total_seconds()
		print(deltaT)
		self.prev_time = datetime.now()
		x,y,x_d,y_d,v,omega = self.feedback_controller(msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.orientation.z,deltaT)
		# print(f'T : {deltaT}\n V: {v}\n omega: {omega}')
		twist.linear.x = v
		twist.angular.z = omega
		self.publisher_.publish(twist)


def main(args=None):
	rclpy.init(args=args)
	follow_point = FollowPoint()
	rclpy.spin(follow_point)
	follow_point.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
