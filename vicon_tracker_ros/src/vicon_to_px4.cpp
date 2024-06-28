#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ViconToPx4 : public rclcpp::Node
{
public:
    ViconToPx4() : Node("vicon_to_px4")
    {
        vicon_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "Hagrid/pose", 10, std::bind(&ViconToPx4::vicon_pose_callback, this, _1));
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr vicon_pose_subscriber_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

    void vicon_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        px4_msgs::msg::TrajectorySetpoint trajectory_setpoint_msg;
        trajectory_setpoint_msg.position = {msg->position.x, msg->position.y, msg->position.z};
        
        // Convert orientation to yaw angle (simplified)
        trajectory_setpoint_msg.yaw = atan2(2.0 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y), 
                                            1.0 - 2.0 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z));
        trajectory_setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(trajectory_setpoint_msg);
    }
};

int main(int argc, char *argv[])
{
    std::cout << "Starting Vicon to PX4 node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ViconToPx4>());
    rclcpp::shutdown();
    return 0;
}
