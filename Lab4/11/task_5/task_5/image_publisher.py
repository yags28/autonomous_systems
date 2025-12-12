import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        # Create publisher
        self.publisher_ = self.create_publisher(Image, '/video_data', 10)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Path to video
        package_share_dir = get_package_share_directory('task_5')
        video_path = os.path.join(package_share_dir, 'resource', 'lab3_video.avi')
        
        # Load video
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open video file: {video_path}')
            return
        
        # Timer callback to publish frames (approx 30 FPS)
        timer_period = 0.03
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Image Publisher node started...')

    def timer_callback(self):
        ret, frame = self.cap.read()

        # If frame not read successfully
        if not ret or frame is None:
            self.get_logger().info('Video playback finished or failed to read frame.')
            self.cap.release()
            self.destroy_timer(self.timer)
            return

        # Safety check
        if not isinstance(frame, np.ndarray):
            self.get_logger().error(f'Frame is not a numpy array! Type: {type(frame)}')
            return

        # Convert to ROS2 Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing video frame...')


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

