#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

class GoalPublisher : public rclcpp::Node
{
public:
    GoalPublisher() : Node("goal_publisher")
    {
        // Subscriber to /robot_pose
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/robot_pose",
            10,
            std::bind(&GoalPublisher::robotPoseCallback, this, std::placeholders::_1)
        );

        // Publisher to /goal_pose
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        RCLCPP_INFO(this->get_logger(), "GoalPublisher node started...");
    }

private:
    void robotPoseCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)  // If red ball detected
        {
            geometry_msgs::msg::PoseStamped goal_msg;

            goal_msg.header.stamp = this->get_clock()->now();
            goal_msg.header.frame_id = "map";

            goal_msg.pose.position.x = -2.447535753250122;
            goal_msg.pose.position.y = 4.189397811889648;
            goal_msg.pose.position.z = 0.0;

            goal_msg.pose.orientation.x = 0.0;
            goal_msg.pose.orientation.y = 0.0;
            goal_msg.pose.orientation.z = -0.7730343097675784;
            goal_msg.pose.orientation.w = 0.6343642139356251;

            publisher_->publish(goal_msg);

            RCLCPP_INFO(this->get_logger(), "Goal published to /goal_pose!");
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPublisher>());
    rclcpp::shutdown();
    return 0;
}

