import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedBallTracker(Node):
    def __init__(self):
        super().__init__('red_ball_tracker')
        self.bridge = CvBridge()

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Speeds
        self.forward_speed = 0.2
        self.turn_speed = 0.3

        self.get_logger().info("Red Ball Tracker Node Initialized")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define red color range
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Find contours of red regions
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cmd = Twist()

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > 500:
                (x, y, radius) = cv2.minEnclosingCircle(largest_contour)
                cx = int(x)
                cy = int(y)

                frame_center = frame.shape[1] // 2
                error_x = cx - frame_center

                # Turn toward the ball
                cmd.angular.z = -float(error_x) / 400.0

                # Forward/backward motion based on ball distance
                if radius < 30:
                    cmd.linear.x = self.forward_speed
                elif radius > 60:
                    cmd.linear.x = -self.forward_speed
                else:
                    cmd.linear.x = 0.0

                self.get_logger().info(f"Ball detected | X:{cx}, Radius:{radius:.1f}, "
                                       f"Lin:{cmd.linear.x:.2f}, Ang:{cmd.angular.z:.2f}")
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
        else:
            self.get_logger().info("No red ball detected — stopping.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RedBallTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

