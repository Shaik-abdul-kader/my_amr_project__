#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile
import math
import tf_transformations
import time


class StraightMover(Node):
    def __init__(self):
        super().__init__('straight_mover')

        # Parameters
        self.declare_parameter('cmd_topic', 'cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.2)
        self.declare_parameter('distance_threshold', 1.5)
        self.declare_parameter('turn_angle_deg', 45.0)
        self.declare_parameter('zigzag_angle_deg', 25.0)
        self.declare_parameter('zigzag_interval', 15.0)

        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.amcl_pose_topic = self.get_parameter('amcl_pose_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.linear_speed = self.get_parameter('linear_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value
        self.distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value
        self.turn_angle_rad = math.radians(self.get_parameter('turn_angle_deg').get_parameter_value().double_value)
        self.zigzag_angle = math.radians(self.get_parameter('zigzag_angle_deg').get_parameter_value().double_value)
        self.zigzag_interval = self.get_parameter('zigzag_interval').get_parameter_value().double_value

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, QoSProfile(depth=10))
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, self.amcl_pose_topic, self.amcl_callback, QoSProfile(depth=10))
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, QoSProfile(depth=10))

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

        # Internal state
        self.localized = False
        self.obstacle_detected = False
        self.current_yaw = 0.0
        self.initial_turn_yaw = None
        self.state = 'STRAIGHT'
        self.last_zigzag_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.zigzag_direction = 'RIGHT'

        self.get_logger().info("StraightMover with zigzag and obstacle avoidance started.")

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        position_cov = cov[0] + cov[7]
        yaw_cov = cov[35]
        if position_cov <= 0.06 and yaw_cov <= 0.1:
            if not self.localized:
                self.get_logger().info("Robot localized. Stopping movement.")
            self.localized = True

    def imu_callback(self, msg: Imu):
        orientation_q = msg.orientation
        quaternion = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ]
        _, _, yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.current_yaw = yaw

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        n = len(ranges)

        def angle_to_index(angle):
            return int((angle - angle_min) / angle_increment)

        start_angle = -math.radians(30)
        end_angle = math.radians(30)
        start_idx = max(0, angle_to_index(start_angle))
        end_idx = min(n - 1, angle_to_index(end_angle))

        scan_window = ranges[start_idx:end_idx + 1]
        self.obstacle_detected = any(
            r < self.distance_threshold for r in scan_window if not math.isinf(r))

    def control_loop(self):
        if self.localized:
            self.publish_stop()
            self.get_logger().info("Localization complete. Stopping.")
            self.timer.cancel()
            return

        # Obstacle detected → enter avoidance mode
        if self.obstacle_detected and self.state != 'AVOIDING':
            self.get_logger().info("Obstacle detected. Entering avoidance mode.")
            self.publish_stop()
            self.state = 'AVOIDING'
            self.initial_turn_yaw = self.current_yaw
            return

        if self.state == 'AVOIDING':
            yaw_diff = self.normalize_angle(self.current_yaw - self.initial_turn_yaw)
            if abs(yaw_diff) >= self.turn_angle_rad:
                self.publish_stop()
                self.get_logger().info("Turned 45°, checking path again.")
                if not self.obstacle_detected:
                    self.get_logger().info("Path clear. Resuming motion.")
                    self.initial_turn_yaw = None
                    self.state = 'STRAIGHT'
                    self.last_zigzag_time = self.get_clock().now().seconds_nanoseconds()[0]
                else:
                    self.initial_turn_yaw = self.current_yaw  # Turn again
            else:
                self.rotate_in_place()
            return

        # Alternate between straight and zigzag
        now = self.get_clock().now().seconds_nanoseconds()[0]
        if self.state == 'STRAIGHT' and (now - self.last_zigzag_time) >= self.zigzag_interval:
            self.state = 'ZIGZAG_RIGHT' if self.zigzag_direction == 'RIGHT' else 'ZIGZAG_LEFT'
            self.last_zigzag_time = now
            self.zigzag_direction = 'LEFT' if self.zigzag_direction == 'RIGHT' else 'RIGHT'

        if self.state == 'STRAIGHT':
            self.move_straight(0.0)

        elif self.state == 'ZIGZAG_RIGHT':
            self.move_straight(-self.zigzag_angle)

        elif self.state == 'ZIGZAG_LEFT':
            self.move_straight(self.zigzag_angle)

    def move_straight(self, angular_offset=0.0):
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = angular_offset
        self.cmd_pub.publish(cmd)

    def rotate_in_place(self):
        cmd = Twist()
        cmd.angular.z = self.angular_speed
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = StraightMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

