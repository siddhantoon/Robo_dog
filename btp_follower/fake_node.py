import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,QoSDurabilityPolicy

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        qos_profile = QoSProfile(
			durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)
        self.publisher_ = self.create_publisher(Odometry, '/odometry', qos_profile)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.publish_odometry)
        # self.i = 0

    def publish_odometry(self):
        x, y, z, quat_x, quat_y, quat_z, quat_w =list(map(float,[0,0,0,0,0,0,0]))
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = quat_x
        msg.pose.pose.orientation.y = quat_y
        msg.pose.pose.orientation.z = quat_z
        msg.pose.pose.orientation.w = quat_w
        print('Pub')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()