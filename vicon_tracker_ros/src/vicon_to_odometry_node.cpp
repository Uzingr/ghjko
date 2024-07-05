#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <cmath>

class ViconToVehicleOdometryNode : public rclcpp::Node
{
public:
    ViconToVehicleOdometryNode()
    : Node("vicon_to_odometry_node")
    {
        // Subscription to the pose topic
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "Hagrid/pose", 10, std::bind(&ViconToVehicleOdometryNode::pose_callback, this, std::placeholders::_1));

        // Publisher to the PX4 vehicle odometry topic
        odom_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("fmu/in/vehicle_visual_odometry", 10);
    }

private:
    void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // Normalize the quaternion
        float norm = std::sqrt(msg->orientation.x * msg->orientation.x +
                               msg->orientation.y * msg->orientation.y +
                               msg->orientation.z * msg->orientation.z +
                               msg->orientation.w * msg->orientation.w);

        if (norm == 0.0) {
            RCLCPP_WARN(this->get_logger(), "Quaternion normalization failed, norm is zero.");
            return;
        }

        // Create and publish a vehicle odometry message
        px4_msgs::msg::VehicleOdometry vo_msg;
        vo_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000; // PX4 expects timestamp in microseconds
        vo_msg.timestamp_sample = vo_msg.timestamp;

        // Set the position and orientation frame of reference
        vo_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

        // Set the position
        vo_msg.position[0] = msg->position.x;
        vo_msg.position[1] = msg->position.y;
        vo_msg.position[2] = msg->position.z;

        // Set the orientation (quaternion)
        vo_msg.q[0] = msg->orientation.w / norm;
        vo_msg.q[1] = msg->orientation.x / norm;
        vo_msg.q[2] = msg->orientation.y / norm;
        vo_msg.q[3] = msg->orientation.z / norm;

        // Log the position and orientation
        RCLCPP_INFO(this->get_logger(), "Publishing odometry: position [%.2f, %.2f, %.2f], orientation [%.2f, %.2f, %.2f, %.2f]",
                    vo_msg.position[0], vo_msg.position[1], vo_msg.position[2], vo_msg.q[0], vo_msg.q[1], vo_msg.q[2], vo_msg.q[3]);

        // Set velocity to NaN since we don't have this data from the Vicon system
        vo_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
        vo_msg.velocity[0] = std::numeric_limits<float>::quiet_NaN();
        vo_msg.velocity[1] = std::numeric_limits<float>::quiet_NaN();
        vo_msg.velocity[2] = std::numeric_limits<float>::quiet_NaN();

        // Set angular velocity to NaN since we don't have this data from the Vicon system
        vo_msg.angular_velocity[0] = std::numeric_limits<float>::quiet_NaN();
        vo_msg.angular_velocity[1] = std::numeric_limits<float>::quiet_NaN();
        vo_msg.angular_velocity[2] = std::numeric_limits<float>::quiet_NaN();

        // Set variance to NaN
        vo_msg.position_variance[0] = std::numeric_limits<float>::quiet_NaN();
        vo_msg.position_variance[1] = std::numeric_limits<float>::quiet_NaN();
        vo_msg.position_variance[2] = std::numeric_limits<float>::quiet_NaN();

        vo_msg.orientation_variance[0] = std::numeric_limits<float>::quiet_NaN();
        vo_msg.orientation_variance[1] = std::numeric_limits<float>::quiet_NaN();
        vo_msg.orientation_variance[2] = std::numeric_limits<float>::quiet_NaN();

        vo_msg.velocity_variance[0] = std::numeric_limits<float>::quiet_NaN();
        vo_msg.velocity_variance[1] = std::numeric_limits<float>::quiet_NaN();
        vo_msg.velocity_variance[2] = std::numeric_limits<float>::quiet_NaN();

        // Set reset counter and quality
        vo_msg.reset_counter = 0;
        vo_msg.quality = 1;  // Set a positive quality value

        // Publish the vehicle odometry message
        odom_publisher_->publish(vo_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ViconToVehicleOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
