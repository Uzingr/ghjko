#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <limits>

class MoCapPubSubNode : public rclcpp::Node
{
public:
    MoCapPubSubNode() : Node("px4_mocap_pubsub")
    {
        // Initialize subscriber to pose topic
        pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/Hagrid/pose", 10, std::bind(&MoCapPubSubNode::pose_callback, this, std::placeholders::_1));

        // Initialize subscriber to PX4 timesync topic
        timesync_sub_ = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
            "/fmu/out/timesync_status", 10, std::bind(&MoCapPubSubNode::timesync_callback, this, std::placeholders::_1));

        // Initialize publisher to PX4 vehicle_visual_odometry topic
        mocap_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);

        timesync_ = 0;
        RCLCPP_INFO(this->get_logger(), "PX4 mocap pub-sub node initialized");
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        auto msg_px4 = px4_msgs::msg::VehicleOdometry();

        msg_px4.timestamp = timesync_;
        msg_px4.timestamp_sample = timesync_;

        msg_px4.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
        msg_px4.position[0] = msg->position.x;
        msg_px4.position[1] = msg->position.y;
        msg_px4.position[2] = msg->position.z;
        msg_px4.q[0] = msg->orientation.w;
        msg_px4.q[1] = msg->orientation.x;
        msg_px4.q[2] = msg->orientation.y;
        msg_px4.q[3] = msg->orientation.z;

        msg_px4.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD;
        msg_px4.velocity[0] = std::numeric_limits<float>::quiet_NaN();
        msg_px4.velocity[1] = std::numeric_limits<float>::quiet_NaN();
        msg_px4.velocity[2] = std::numeric_limits<float>::quiet_NaN();
        msg_px4.angular_velocity[0] = std::numeric_limits<float>::quiet_NaN();
        msg_px4.angular_velocity[1] = std::numeric_limits<float>::quiet_NaN();
        msg_px4.angular_velocity[2] = std::numeric_limits<float>::quiet_NaN();

        msg_px4.position_variance[0] = 0.0;
        msg_px4.position_variance[1] = 0.0;
        msg_px4.position_variance[2] = 0.0;
        msg_px4.orientation_variance[0] = 0.0;
        msg_px4.orientation_variance[1] = 0.0;
        msg_px4.orientation_variance[2] = 0.0;
        msg_px4.velocity_variance[0] = 0.0;
        msg_px4.velocity_variance[1] = 0.0;
        msg_px4.velocity_variance[2] = 0.0;

        mocap_pub_->publish(msg_px4);
    }

    void timesync_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
    {
        timesync_ = msg->timestamp;
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr timesync_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr mocap_pub_;
    uint64_t timesync_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoCapPubSubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
