import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/webcam', 10)
        self.bridge = CvBridge()
        
        # Set up the camera
        self.cap = cv2.VideoCapture(0)

        # Start publishing the camera feed
        self.publish_camera_feed()

    def publish_camera_feed(self):
        while True:
            # Capture a frame from the camera
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Unable to capture frame from camera!")
                continue

            # Convert the frame to a ROS2 Image message and publish it
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            # cv2.imshow('cam',frame)
            # cv2.waitKey(1)
            self.publisher_.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    rclpy.spin(camera_publisher)

    camera_publisher.cap.release()
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
