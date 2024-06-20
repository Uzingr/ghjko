#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

class ViconToOdometryNode : public rclcpp::Node
{
public:
    ViconToOdometryNode()
    : Node("vicon_to_odometry_node")
    {
        // Subscription to the pose topic
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "Hagrid/pose", 10, std::bind(&ViconToOdometryNode::pose_callback, this, std::placeholders::_1));

        // Publisher to the odometry topic
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/fmu/out/pose_to_odometry", 10);
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // Create and publish an odometry message
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Set the position
        odom_msg.pose.pose.position.x = msg->position.x;
        odom_msg.pose.pose.position.y = msg->position.y;
        odom_msg.pose.pose.position.z = msg->position.z;

        // Set the orientation
        odom_msg.pose.pose.orientation.x = msg->orientation.x;
        odom_msg.pose.pose.orientation.y = msg->orientation.y;
        odom_msg.pose.pose.orientation.z = msg->orientation.z;
        odom_msg.pose.pose.orientation.w = msg->orientation.w;

        // Publish the odometry message
        odom_publisher_->publish(odom_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ViconToOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
