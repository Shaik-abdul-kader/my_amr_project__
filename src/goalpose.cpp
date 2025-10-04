// navfn_path_follower.cpp
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class NavfnPathFollower : public rclcpp::Node
{
public:
  using ComputePathAction = nav2_msgs::action::ComputePathToPose;
  using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathAction>;

  NavfnPathFollower()
  : Node("navfn_path_follower"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // publisher to /cmd_vel
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // action client to the planner server's ComputePathToPose (usually '/compute_path_to_pose')
    action_client_ = rclcpp_action::create_client<ComputePathAction>(
      this, "compute_path_to_pose");

    // parameters: goal pose in map frame (user will edit these)
    this->declare_parameter<double>("goal_x", 1.0);
    this->declare_parameter<double>("goal_y", 0.0);
    this->declare_parameter<double>("goal_yaw", 0.0);
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<double>("goal_tolerance", 0.15);
    this->declare_parameter<double>("waypoint_tolerance", 0.25);
    this->declare_parameter<double>("linear_gain", 0.8);
    this->declare_parameter<double>("angular_gain", 1.5);
    this->declare_parameter<double>("max_linear", 0.35);
    this->declare_parameter<double>("max_angular", 1.0);

    getParams();

    // small delay then start
    timer_ = this->create_wall_timer(1s, std::bind(&NavfnPathFollower::start_sequence, this));
  }

private:
  void getParams()
  {
    map_frame_ = this->get_parameter("map_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    goal_x_ = this->get_parameter("goal_x").as_double();
    goal_y_ = this->get_parameter("goal_y").as_double();
    goal_yaw_ = this->get_parameter("goal_yaw").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
    waypoint_tolerance_ = this->get_parameter("waypoint_tolerance").as_double();
    linear_gain_ = this->get_parameter("linear_gain").as_double();
    angular_gain_ = this->get_parameter("angular_gain").as_double();
    max_linear_ = this->get_parameter("max_linear").as_double();
    max_angular_ = this->get_parameter("max_angular").as_double();
  }

  void start_sequence()
  {
    // stop timer so we only call once
    timer_->cancel();

    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "ComputePathToPose action server not available after 5s");
      return;
    }

    // Build goal pose
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = map_frame_;
    goal.header.stamp = this->now();
    goal.pose.position.x = goal_x_;
    goal.pose.position.y = goal_y_;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_yaw_);
    goal.pose.orientation = tf2::toMsg(q);

    // Create action goal
    auto goal_msg = ComputePathAction::Goal();
    goal_msg.goal = goal;
    goal_msg.planner_id = "NavfnPlanner";
    // optional: set planner_id here if you need a specific planner plugin name
    // e.g. goal_msg.planner_id = "NavfnPlanner"; // if configured in planner_server

    RCLCPP_INFO(this->get_logger(), "Sending compute_path_to_pose goal to planner...");
    auto send_goal_options = rclcpp_action::Client<ComputePathAction>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&NavfnPathFollower::on_compute_path_result, this, std::placeholders::_1);
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void on_compute_path_result(const GoalHandleComputePath::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(this->get_logger(), "Planner returned path, length: %zu", result.result->path.poses.size());
      path_ = result.result->path;
      // start path follow loop
      follow_path_timer_ = this->create_wall_timer(100ms, std::bind(&NavfnPathFollower::follow_path_step, this));
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to compute path (action result code %d)", (int)result.code);
    }
  }

  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose_out)
  {
    try {
      geometry_msgs::msg::TransformStamped t = tf_buffer_.lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero, 500ms);
      pose_out.header.frame_id = map_frame_;
      pose_out.header.stamp = this->now();
      pose_out.pose.position.x = t.transform.translation.x;
      pose_out.pose.position.y = t.transform.translation.y;
      pose_out.pose.position.z = t.transform.translation.z;
      pose_out.pose.orientation = t.transform.rotation;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return false;
    }
  }

  void follow_path_step()
  {
    if (path_.poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "No path to follow");
      return;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      // can't read TF; stop robot for safety
      publish_stop();
      return;
    }

    // find current target waypoint index (advance when within tolerance)
    // keep waypoint_index_ within range
    if (waypoint_index_ >= path_.poses.size()) {
      RCLCPP_INFO(this->get_logger(), "Reached end of path");
      publish_stop();
      follow_path_timer_->cancel();
      return;
    }

    // target pose
    const auto & target = path_.poses[waypoint_index_];

    // compute dx, dy in map frame
    double dx = target.pose.position.x - robot_pose.pose.position.x;
    double dy = target.pose.position.y - robot_pose.pose.position.y;
    double distance = std::hypot(dx, dy);

    // compute robot yaw
    tf2::Quaternion q(robot_pose.pose.orientation.x,
                      robot_pose.pose.orientation.y,
                      robot_pose.pose.orientation.z,
                      robot_pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double target_yaw = 0.0;
    // compute heading to the waypoint
    target_yaw = std::atan2(dy, dx);

    double yaw_err = normalize_angle(target_yaw - yaw);

    // if close to current waypoint, advance
    if (distance < waypoint_tolerance_) {
      waypoint_index_++;
      RCLCPP_DEBUG(this->get_logger(), "Advancing to waypoint %zu", waypoint_index_);
      // if finished all waypoints, check final goal tolerance
      if (waypoint_index_ >= path_.poses.size()) {
        double final_goal_dist = std::hypot(
          path_.poses.back().pose.position.x - robot_pose.pose.position.x,
          path_.poses.back().pose.position.y - robot_pose.pose.position.y);
        if (final_goal_dist <= goal_tolerance_) {
          RCLCPP_INFO(this->get_logger(), "Goal reached within tolerance (%.3f m). Stopping.", final_goal_dist);
          publish_stop();
          follow_path_timer_->cancel();
        }
      }
      return;
    }

    // simple proportional controller: linear velocity depends on distance, angular on yaw_error
    double linear = linear_gain_ * distance;
    double angular = angular_gain_ * yaw_err;

    // limit
    linear = std::clamp(linear, -max_linear_, max_linear_);
    angular = std::clamp(angular, -max_angular_, max_angular_);

    // if yaw error large, reduce forward speed
    if (std::fabs(yaw_err) > 0.6) {
      linear = 0.0;
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    cmd_pub_->publish(cmd);
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist t;
    t.linear.x = 0.0;
    t.angular.z = 0.0;
    cmd_pub_->publish(t);
  }

  static double normalize_angle(double a)
  {
    while (a > M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
  }

private:
  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  // action client
  rclcpp_action::Client<ComputePathAction>::SharedPtr action_client_;

  // timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr follow_path_timer_;

  // path and index
  nav_msgs::msg::Path path_;
  size_t waypoint_index_ = 0;

  // parameters
  std::string map_frame_, base_frame_;
  double goal_x_, goal_y_, goal_yaw_;
  double goal_tolerance_, waypoint_tolerance_;
  double linear_gain_, angular_gain_;
  double max_linear_, max_angular_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavfnPathFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


