#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool  # Using standard Bool message
from cv_bridge import CvBridge
import cv2
import numpy as np


class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')

        # Subscriber to camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )

        # Publisher for robot_pose (True if red ball detected)
        self.publisher_ = self.create_publisher(Bool, '/robot_pose', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Ball Detector Node Started...")

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Red color mask
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = False
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:
                perimeter = cv2.arcLength(cnt, True)
                if perimeter == 0:
                    continue
                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                if circularity > 0.7:
                    (x, y), radius = cv2.minEnclosingCircle(cnt)
                    if radius > 10:
                        detected = True
                        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)

        # Publish True/False to /robot_pose
        msg_out = Bool()
        msg_out.data = detected
        self.publisher_.publish(msg_out)

        if detected:
            self.get_logger().info("SUCCESS: Red ball detected!")

        # Debug windows
        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

