import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from datetime import datetime

class OdomSubscriber(Node):

	def __init__(self):
		super().__init__('odom_subscriber')
		self.subscription = self.create_subscription(
			Odometry,
			'/odom',
			self.odom_callback,
			10)
		self.subscription  # prevent unused variable warning
		qos_profile = QoSProfile(
			depth=10
		)
		self.publisher_ = self.create_publisher(Twist, '/cmd_vel', qos_profile)
		self.initial_time = datetime.now()
		self.prev_time = datetime.now()
		self.theta = 0.0
		self.omega = 0.0
		self.v = 0.0
		self.t = 0.0
		timer_period = 0.1# seconds
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.msg = Odometry()

	
	def timer_callback(self):
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

	def odom_callback(self, msg):
		# self.get_logger().info('Position: [x: {}, y: {}, z: {}]'.format(
		# 	msg.pose.pose.position.x,
		# 	msg.pose.pose.position.y,
		# 	msg.pose.pose.position.z))
		self.msg = msg
		
	def sliding_controller(self, x, y,theta, deltaT):
		# theta = self.theta

		L = 0.2
		T = deltaT #0.01
		k1 = 1
		k2 = 1
		w = 0.05
		r = 1.0
		
		t = (self.prev_time - self.initial_time).total_seconds()
		print(f't :{t}')
		
		# x_d = r*np.cos(w*t)
		# y_d = r*np.sin(w*t)
		# xddot = -w*r* np.sin(w * t)
		# yddot = w *r* np.cos(w * t)
		
		x_d = 5
		# y_d = r*np.sin(w*t)
		y_d = 0
		xddot = 0
		# yddot = w *r* np.cos(w * t)
		yddot = 0
		
		ex = x - x_d
		ey = y - y_d
		v = -(k1) * np.cos(theta) * np.tanh(ex) - (k2) * np.sin(theta) * np.tanh(ey) + (xddot) * np.cos(theta) + (yddot) * np.sin(theta)
		omega = (k1 / L) * np.sin(theta) * np.tanh(ex) - (k2 / L) * np.cos(theta) * np.tanh(ey) - (xddot / L) * np.sin(theta) + (yddot / L) * np.cos(theta)
		
		# if omega>2.0:
		# 	omega=2.0
		# if omega<-2.0:
		# 	omega=-2.0
		# if v>1.0:
		# 	v=1.0
		# if v<-1.0:
		# 	v=-1.0

		expected_theta = theta + omega*T
		print(f'Expected:\n theta:{expected_theta}\n V: {v}\n omega: {omega}\nError: \n ex:{ex}\n ey:{ey}')
		# self.v = v
		# self.omega = omega

		return x,y,x_d,y_d,v,omega
		
	def feedback_controller(self, x, y,theta,deltaT):
		L = 0.1
		T = deltaT #0.01
		k1 = 0.2
		k2 = 0.5
		w = 0.12
		r = 1.0
		t = (self.prev_time - self.initial_time).total_seconds()
		
		# x_d = np.cos(w*t)
		# y_d = np.sin(w*t)
		# xddot = -w*np.sin(w*t)
		# yddot = w*np.cos(w*t)

		# x-axis
		x_d = 5
		y_d = 2
		# y_d =  r*np.sin(w*t)
		xddot = 0
		# yddot = w *r* np.cos(w * t)
		yddot = 0

		# y axis
		# x_d = r*np.sin(w*t)
		# y_d =  0.1*t 
		# # y_d = 0
		# xddot = w *r* np.cos(w * t)
		# yddot = 0.1
		# # yddot = 0

		ex = x_d - x
		ey = y_d - y
		v = (k1)*np.cos(theta)*ex + (k2)*np.sin(theta)*ey + (xddot)*np.cos(theta) + (yddot)*np.sin(theta)
		omega = -(k1/L)*np.sin(theta)*ex + (k2/L)*np.cos(theta)*ey - (xddot/L)*np.sin(theta) + (yddot/L)*np.cos(theta)
		expected_theta = theta + omega*T
		OM_MAX = 1.0
		if omega>OM_MAX:
			omega=OM_MAX
		if omega<-OM_MAX:
			omega=-OM_MAX
		V_MAX = 0.1
		if v>V_MAX:
			v=V_MAX
		if v<-V_MAX:
			v=-V_MAX
		print(f'Expected:\n theta:{expected_theta}\n V: {v}\n omega: {omega}\nError: \n ex:{ex}\n ey:{ey}')

		return x,y,x_d,y_d,v,omega
	
	


def main(args=None):
	rclpy.init(args=args)

	odom_subscriber = OdomSubscriber()

	rclpy.spin(odom_subscriber)

	odom_subscriber.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

# Callback to subscribe to odometry
# Read odometry store in self.x and self.y 
# create a function controller call from callback
# controller takes x, y, deltaT and publishes the V as twist.linear.x & omega as twist.angular.z



