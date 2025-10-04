#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
import math
import tf_transformations
from tf2_ros import Buffer, TransformListener, LookupException

class NavfnPathFollower(Node):
    def __init__(self):
        super().__init__('navfn_path_follower')

        # Parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('waypoint_tolerance', 0.15)
        self.declare_parameter('linear_gain', 0.8)
        self.declare_parameter('angular_gain', 1.5)
        self.declare_parameter('max_linear', 0.35)
        self.declare_parameter('max_angular', 1.0)

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        self.linear_gain = self.get_parameter('linear_gain').get_parameter_value().double_value
        self.angular_gain = self.get_parameter('angular_gain').get_parameter_value().double_value
        self.max_linear = self.get_parameter('max_linear').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular').get_parameter_value().double_value

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        # Subscribe to /goal_pose from RViz
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Path data
        self.path = None
        self.waypoint_index = 0
        self.follow_timer = None

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info(f'Received goal: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}')
        self.send_goal_to_planner(msg)

    def send_goal_to_planner(self, goal_pose: PoseStamped):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('ComputePathToPose action server not available')
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal_pose
        goal_msg.planner_id = "GridBased"

        self.get_logger().info('Sending goal to planner...')
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by planner')
            return
        self.get_logger().info('Goal accepted. Waiting for path...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.path_result_callback)

    def path_result_callback(self, future):
        result = future.result().result
        if not result or not result.path or len(result.path.poses) == 0:
            self.get_logger().error('No valid path received')
            return

        self.path = result.path
        self.waypoint_index = 0
        self.get_logger().info(f'Path received with {len(self.path.poses)} waypoints')

        # Start following the path
        if self.follow_timer is None:
            self.follow_timer = self.create_timer(0.1, self.follow_path_step)

    def follow_path_step(self):
        if not self.path or self.waypoint_index >= len(self.path.poses):
            self.publish_stop()
            self.path = None
            if self.follow_timer:
                self.follow_timer.cancel()
                self.follow_timer = None
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.publish_stop()
            return

        target = self.path.poses[self.waypoint_index]
        dx = target.pose.position.x - robot_pose.pose.position.x
        dy = target.pose.position.y - robot_pose.pose.position.y
        distance = math.hypot(dx, dy)

        _, _, yaw = tf_transformations.euler_from_quaternion([
            robot_pose.pose.orientation.x,
            robot_pose.pose.orientation.y,
            robot_pose.pose.orientation.z,
            robot_pose.pose.orientation.w
        ])

        target_yaw = math.atan2(dy, dx)
        yaw_err = self.normalize_angle(target_yaw - yaw)

        # Advance to next waypoint
        if distance < self.waypoint_tolerance:
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.path.poses):
                final_goal_dist = math.hypot(
                    self.path.poses[-1].pose.position.x - robot_pose.pose.position.x,
                    self.path.poses[-1].pose.position.y - robot_pose.pose.position.y
                )
                if final_goal_dist <= self.goal_tolerance:
                    self.get_logger().info(f'Goal reached within {self.goal_tolerance} m. Stopping.')
                    self.publish_stop()
                    # Cancel timer and clear path
                    if self.follow_timer:
                        self.follow_timer.cancel()
                        self.follow_timer = None
                    self.path = None
                return

        # Simple proportional controller
        linear = self.linear_gain * distance
        angular = self.angular_gain * yaw_err
        if abs(yaw_err) > 0.6:
            linear = 0.0

        # Limit speeds
        linear = max(min(linear, self.max_linear), -self.max_linear)
        angular = max(min(angular, self.max_angular), -self.max_angular)

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation
            return pose
        except LookupException:
            self.get_logger().warn('TF lookup failed')
            return None

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
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = NavfnPathFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

