#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <Eigen/Dense>

class AngularVelocityNode : public rclcpp::Node
{
public:
    AngularVelocityNode()
    : Node("angular_velocity_node"), has_prev_pose_(false)
    {
        // Subscriber to /odom_gt (ground truth odometry)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_gt", 10, std::bind(&AngularVelocityNode::odomCallback, this, std::placeholders::_1));

        // Publisher for finite difference angular velocity
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_gt_fd", 10);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Get the current time
        rclcpp::Time current_time = msg->header.stamp;

        // Get the current position and orientation from the Odometry message
        Eigen::Vector3d position(msg->pose.pose.position.x,
                                 msg->pose.pose.position.y,
                                 msg->pose.pose.position.z);

        Eigen::Quaterniond q_current(msg->pose.pose.orientation.w,
                                     msg->pose.pose.orientation.x,
                                     msg->pose.pose.orientation.y,
                                     msg->pose.pose.orientation.z);

        // Construct the SE3 object for the current pose
        pinocchio::SE3 current_pose(q_current.toRotationMatrix(), position);

        // Check if we have a previous pose to compute the finite difference
        if (!has_prev_pose_)
        {
            prev_pose_ = current_pose;
            prev_time_ = current_time;
            has_prev_pose_ = true;
            return;
        }

        // Compute time difference between consecutive poses
        double dt = (current_time - prev_time_).seconds();
        if (dt <= 0.0) return;  // Avoid division by zero or invalid dt

        // Compute the relative transformation between previous and current pose using actInv
        pinocchio::SE3 relative_transform = prev_pose_.actInv(current_pose);

        // Compute the logarithmic map to obtain the twist (contains linear and angular velocities)
        pinocchio::Motion twist_fd = pinocchio::log6(relative_transform) / dt;

        // Angular velocity is the rotational part of the twist divided by dt
        Eigen::Vector3d angular_velocity_world = twist_fd.angular();

        // Create an IMU message and populate it with the computed angular velocity
        auto imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = current_time;
        imu_msg.header.frame_id = msg->header.frame_id;  // Use the same frame as the input Odometry

        // Set the angular velocity
        imu_msg.angular_velocity.x = angular_velocity_world.x();
        imu_msg.angular_velocity.y = angular_velocity_world.y();
        imu_msg.angular_velocity.z = angular_velocity_world.z();

        // Publish the IMU message with the finite difference angular velocity
        imu_pub_->publish(imu_msg);

        // Update previous pose and time for the next callback
        prev_pose_ = current_pose;
        prev_time_ = current_time;
    }

    // ROS2 publishers and subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // Previous SE3 pose and timestamp
    pinocchio::SE3 prev_pose_;
    rclcpp::Time prev_time_;
    bool has_prev_pose_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AngularVelocityNode>());
    rclcpp::shutdown();
    return 0;
}
