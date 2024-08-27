#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>


class OdomPublisherNode : public rclcpp::Node
{
public:
    OdomPublisherNode()
        : Node("odom_publisher_node")
    {
        // Initialize subscriber to "base_lin_vel" topic
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/base_lin_vel", 10, std::bind(&OdomPublisherNode::twist_callback, this, std::placeholders::_1));

        // Initialize publisher for odometry on "/odom_kinematics" topic
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_kinematics", 10);

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Initialize the odometry message
        odom_msg_.header.frame_id = "odom_kinematics";      // Odometry frame
        odom_msg_.child_frame_id = "base_link";  // Robot base frame

        // Initialize position and orientation
        x_ = -2.0;
        y_ = 3.5;
        z_ = 0.3;
        // roll_ = 0.026;
        pitch_ = 0.861;
        // yaw_ =  1.403;
        roll_ = 0.0;
        // pitch_ = 0.0;
        yaw_ = 0.0;

        prev_rot_mat_ = tf2::Matrix3x3::getIdentity();
        orientation_matrix_.setIdentity();
        orientation_matrix_.setRPY(roll_, pitch_, yaw_);

        // Initialize previous time
        last_time_ = this->get_clock()->now();
    }

private:
    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        // Get the current time
        rclcpp::Time current_time = this->get_clock()->now();

        // Compute the time difference
        double dt = (current_time - last_time_).seconds();

        // Extract linear and angular velocities
        double vx_body = msg->twist.linear.x;
        double vy_body = msg->twist.linear.y;
        double vz_body = msg->twist.linear.z;
        double omega_x = msg->twist.angular.x;
        double omega_y = msg->twist.angular.y;
        double omega_z = msg->twist.angular.z;

        // Create a small rotation quaternion based on the angular velocities (in the body frame)
        tf2::Quaternion delta_q;
        double magnitude = std::sqrt(omega_x * omega_x + omega_y * omega_y + omega_z * omega_z);
        if (magnitude > 1e-12) {
            double half_theta = 0.5 * magnitude * dt;
            double sin_half_theta = std::sin(half_theta);
            delta_q.setValue(sin_half_theta * omega_x / magnitude,
                             sin_half_theta * omega_y / magnitude,
                             sin_half_theta * omega_z / magnitude,
                             std::cos(half_theta));
        } else {
            // No rotation, or very small angular velocity
            delta_q.setValue(0, 0, 0, 1);
        }

        // Convert the small rotation quaternion to a rotation matrix
        tf2::Matrix3x3 delta_matrix(delta_q);

        // Update the current orientation matrix by applying the small rotation (body frame to world frame)
        orientation_matrix_ *= delta_matrix;

        // Convert the current orientation matrix back to roll, pitch, yaw
        double roll, pitch, yaw;
        orientation_matrix_.getRPY(roll, pitch, yaw);

        // Convert body frame velocities to world frame using the current orientation matrix
        tf2::Vector3 velocity_body(vx_body, vy_body, vz_body);
        tf2::Vector3 velocity_world = orientation_matrix_ * velocity_body;

        // Update position based on world frame velocities
        x_ += velocity_world.getX() * dt;
        y_ += velocity_world.getY() * dt;
        z_ += velocity_world.getZ() * dt;

        // Fill in the odometry message
        odom_msg_.header.stamp = msg->header.stamp;

        // Position
        odom_msg_.pose.pose.position.x = x_;
        odom_msg_.pose.pose.position.y = y_;
        odom_msg_.pose.pose.position.z = z_;

        // Orientation (as quaternion)
        tf2::Quaternion q;
        orientation_matrix_.getRotation(q);
        odom_msg_.pose.pose.orientation = tf2::toMsg(q);

        // Set the velocity in the odometry message (in the body frame)
        odom_msg_.twist.twist.linear.x = vx_body;
        odom_msg_.twist.twist.linear.y = vy_body;
        odom_msg_.twist.twist.linear.z = vz_body;
        odom_msg_.twist.twist.angular.x = omega_x;
        odom_msg_.twist.twist.angular.y = omega_y;
        odom_msg_.twist.twist.angular.z = omega_z;

        // Publish the odometry message
        odom_publisher_->publish(odom_msg_);

        // Publish the transform from odom_kinematics to base_link
        geometry_msgs::msg::TransformStamped odom_to_base_link;
        odom_to_base_link.header.stamp = msg->header.stamp;
        odom_to_base_link.header.frame_id = "odom_kinematics";
        odom_to_base_link.child_frame_id = "base_link";

        odom_to_base_link.transform.translation.x = x_;
        odom_to_base_link.transform.translation.y = y_;
        odom_to_base_link.transform.translation.z = z_;
        odom_to_base_link.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(odom_to_base_link);

        // Update the last time
        last_time_ = current_time;




        // // Rotate the angular velocity from the body frame to the world frame
        // tf2::Vector3 omega_body(omega_x, omega_y, omega_z);
        // tf2::Vector3 omega_world = prev_rot_mat_ * omega_body;
        // // tf2::Vector3 omega_world = omega_body;

        // // Integrate angular velocity to update roll, pitch, and yaw (in the world frame)
        // roll_ += omega_world.getX() * dt;
        // pitch_ += omega_world.getY() * dt;
        // yaw_ += omega_world.getZ() * dt;

        // // Update the rotation matrix with the new roll, pitch, and yaw
        // prev_rot_mat_.setRPY(roll_, pitch_, yaw_);

        // // Convert body frame velocities to world frame using the updated rotation matrix
        // tf2::Vector3 velocity_body(vx_body, vy_body, vz_body);
        // tf2::Vector3 velocity_world = prev_rot_mat_ * velocity_body;

        // // Update position based on world frame velocities
        // x_ += velocity_world.getX() * dt;
        // y_ += velocity_world.getY() * dt;
        // z_ += velocity_world.getZ() * dt;

        // // roll_ += omega_world.getX() * dt;
        // // pitch_ += omega_world.getY() * dt;
        // // yaw_ += omega_world.getZ() * dt;
        // // prev_rot_mat_.setRPY(roll_, pitch_, yaw_);

        // // Fill in the odometry message
        // odom_msg_.header.stamp = msg->header.stamp;

        // // Position
        // odom_msg_.pose.pose.position.x = x_;
        // odom_msg_.pose.pose.position.y = y_;
        // odom_msg_.pose.pose.position.z = z_;

        // // Orientation (as quaternion)
        // tf2::Quaternion q;
        // prev_rot_mat_.getRotation(q);
        // odom_msg_.pose.pose.orientation = tf2::toMsg(q);

        // // Set the velocity in the odometry message (in the body frame)
        // odom_msg_.twist.twist.linear.x = vx_body;
        // odom_msg_.twist.twist.linear.y = vy_body;
        // odom_msg_.twist.twist.linear.z = vz_body;
        // odom_msg_.twist.twist.angular.x = omega_x;
        // odom_msg_.twist.twist.angular.y = omega_y;
        // odom_msg_.twist.twist.angular.z = omega_z;

        // // Publish the odometry message
        // odom_publisher_->publish(odom_msg_);

        // // Publish the transform from odom_kinematics to base_link
        // geometry_msgs::msg::TransformStamped odom_to_base_link;
        // odom_to_base_link.header.stamp = msg->header.stamp;
        // odom_to_base_link.header.frame_id = "odom_kinematics";
        // odom_to_base_link.child_frame_id = "base_link";

        // odom_to_base_link.transform.translation.x = x_;
        // odom_to_base_link.transform.translation.y = y_;
        // odom_to_base_link.transform.translation.z = z_;
        // odom_to_base_link.transform.rotation = tf2::toMsg(q);

        // tf_broadcaster_->sendTransform(odom_to_base_link);

        // // Update the last time
        // last_time_ = current_time;




        // // Convert the current orientation (roll_, pitch_, yaw_) to a quaternion
        // tf2::Quaternion q;
        // q.setRPY(roll_, pitch_, yaw_);

        // // Convert the quaternion to a 3x3 rotation matrix
        // tf2::Matrix3x3 rot_matrix(q);

        // // Convert body frame velocities to world frame using the rotation matrix
        // tf2::Vector3 velocity_body(vx_body, vy_body, vz_body);
        // tf2::Vector3 velocity_world = rot_matrix * velocity_body;

        // // Update position based on world frame velocities
        // x_ += velocity_world.getX() * dt;
        // y_ += velocity_world.getY() * dt;
        // z_ += velocity_world.getZ() * dt;

        // // Update orientation based on angular velocities (integrating angular rates)
        // roll_ += omega_x * dt;
        // pitch_ += omega_y * dt;
        // yaw_ += omega_z * dt;

        // prev_rot_mat = rot_matrix;

        // // Position
        // odom_msg_.pose.pose.position.x = x_;
        // odom_msg_.pose.pose.position.y = y_;
        // odom_msg_.pose.pose.position.z = z_;

        // // Orientation (as quaternion)
        // q.setRPY(roll_, pitch_, yaw_);
        // odom_msg_.pose.pose.orientation = tf2::toMsg(q);

        // // Set the velocity in the odometry message (in the body frame)
        // odom_msg_.twist.twist.linear.x = vx_body;
        // odom_msg_.twist.twist.linear.y = vy_body;
        // odom_msg_.twist.twist.linear.z = vz_body;
        // odom_msg_.twist.twist.angular.x = omega_x;
        // odom_msg_.twist.twist.angular.y = omega_y;
        // odom_msg_.twist.twist.angular.z = omega_z;

        // // Publish the odometry message
        // odom_publisher_->publish(odom_msg_);

        // // Publish the transform from odom_kinematics to base_link
        // geometry_msgs::msg::TransformStamped odom_to_base_link;
        // odom_to_base_link.header.stamp = msg->header.stamp;
        // odom_to_base_link.header.frame_id = "odom_kinematics";
        // odom_to_base_link.child_frame_id = "base_link";

        // odom_to_base_link.transform.translation.x = x_;
        // odom_to_base_link.transform.translation.y = y_;
        // odom_to_base_link.transform.translation.z = z_;
        // odom_to_base_link.transform.rotation = tf2::toMsg(q);

        // tf_broadcaster_->sendTransform(odom_to_base_link);

        // // Update the last time
        // last_time_ = current_time;




        // // Update position based on linear velocities
        // x_ += vx * dt;
        // y_ += vy * dt;
        // z_ += vz * dt;

        // // Update orientation based on angular velocities (integrating angular rates)
        // roll_ += omega_x * dt;
        // pitch_ += omega_y * dt;
        // yaw_ += omega_z * dt;

        // // Fill in the odometry message
        // odom_msg_.header.stamp = msg->header.stamp;

        // // Position
        // odom_msg_.pose.pose.position.x = x_;
        // odom_msg_.pose.pose.position.y = y_;
        // odom_msg_.pose.pose.position.z = z_;

        // // Orientation (as quaternion)
        // tf2::Quaternion q;
        // q.setRPY(roll_, pitch_, yaw_);
        // odom_msg_.pose.pose.orientation = tf2::toMsg(q);

        // // Set the velocity in the odometry message
        // odom_msg_.twist.twist.linear.x = vx;
        // odom_msg_.twist.twist.linear.y = vy;
        // odom_msg_.twist.twist.linear.z = vz;
        // odom_msg_.twist.twist.angular.x = omega_x;
        // odom_msg_.twist.twist.angular.y = omega_y;
        // odom_msg_.twist.twist.angular.z = omega_z;

        // // Publish the odometry message
        // odom_publisher_->publish(odom_msg_);

        // // Publish the transform from odom to base_link
        // geometry_msgs::msg::TransformStamped odom_to_base_link;
        // odom_to_base_link.header.stamp = msg->header.stamp;
        // odom_to_base_link.header.frame_id = "odom_kinematics";
        // odom_to_base_link.child_frame_id = "base_link";

        // odom_to_base_link.transform.translation.x = x_;
        // odom_to_base_link.transform.translation.y = y_;
        // odom_to_base_link.transform.translation.z = z_;
        // odom_to_base_link.transform.rotation = tf2::toMsg(q);

        // tf_broadcaster_->sendTransform(odom_to_base_link);

        // // Update the last time
        // last_time_ = current_time;
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Odometry odom_msg_;

    // Odometry state variables
    double x_;
    double y_;
    double z_;
    double roll_;
    double pitch_;
    double yaw_;
    tf2::Matrix3x3 prev_rot_mat_;
    tf2::Matrix3x3 orientation_matrix_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
