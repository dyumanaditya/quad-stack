#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pinocchio/spatial/se3.hpp>
#include "pinocchio/spatial/explog.hpp"
#include <pinocchio/spatial/motion.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class OdomPublisherNode : public rclcpp::Node
{
public:
    OdomPublisherNode()
        : Node("odom_publisher_node")
    {
        // Initialize subscriber to "base_lin_vel" topic
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/base_lin_vel", 10, std::bind(&OdomPublisherNode::twist_callback, this, std::placeholders::_1));

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/out", 10, std::bind(&OdomPublisherNode::imu_callback, this, std::placeholders::_1));

        // gt_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     "/odom_gt", 10, std::bind(&OdomPublisherNode::odom_gt_callback, this, std::placeholders::_1));

        robot = this->declare_parameter("robot", "");

        // Initialize publisher for odometry on "/odom_kinematics" topic
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_kinematics", 10);

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Initialize the odometry message
        odom_msg_.header.frame_id = "odom_kinematics";      // Odometry frame

        if (robot == "silver_badger" || robot == "honey_badger")
        {
            base_frame_name_ = "base_link";  // Robot base frame
        }
        else
        {
            base_frame_name_ = "base";  // Robot base frame
        }
        odom_msg_.child_frame_id = base_frame_name_;

        // Initialize position and orientation
        first_time_ = true;
        // x_ = -2.0497805745819044;
        // y_ = 3.501646823445426;
        // z_ = 0.2930235459349441;
        // roll_ =  -0.0008285592586317505;
        // pitch_ = -0.035986024291907226;
        // yaw_ =   0.0069288823779056225;
        
        // x_ = -2.0459031821872986;
        // y_ = 3.496181887042353;
        // z_ = 0.3003354309985301;
        // roll_ =  0.036;
        // pitch_ = -1.604;
        // yaw_ =   1.596;
        // x_ = -2.0;
        // y_ = 3.5;
        // z_ = 0.05;
        x_ = 0.0;
        y_ = 0.0;
        z_ = 0.0;
        roll_ = 0.0;
        pitch_ = 0.0;
        yaw_ = 0.0;

        // double deg_to_rad = M_PI / 180.0;
        // roll_ *= deg_to_rad;
        // pitch_ *= deg_to_rad;
        // yaw_ *= deg_to_rad;

        // Create translation vector
        Eigen::Vector3d initial_translation(x_, y_, z_);

        tf2::Quaternion tf_q;
        tf_q.setRPY(roll_, pitch_, yaw_);  // Assumes roll, pitch, yaw are in radians

        // publish_odometry(x_, y_, z_, tf_q, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, this->get_clock()->now());

        // Convert tf2 quaternion to Eigen quaternion
        Eigen::Quaterniond eigen_q(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());  // Note the order

        // Convert Eigen quaternion to rotation matrix
        Eigen::Matrix3d initial_rotation = eigen_q.toRotationMatrix();

        // Create rotation matrix from roll_, pitch_, yaw_
        // Eigen::AngleAxisd rollAngle(roll_, Eigen::Vector3d::UnitX());
        // Eigen::AngleAxisd pitchAngle(pitch_, Eigen::Vector3d::UnitY());
        // Eigen::AngleAxisd yawAngle(yaw_, Eigen::Vector3d::UnitZ());
        // Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        // Eigen::Matrix3d initial_rotation = q.toRotationMatrix();

        // Initialize pose_ as an SE3 object
        pose_ = pinocchio::SE3(initial_rotation, initial_translation);


        prev_rot_mat_ = tf2::Matrix3x3::getIdentity();
        prev_rot_mat_.setRPY(roll_, pitch_, yaw_);
        orientation_matrix_.setIdentity();
        orientation_matrix_.setRPY(roll_, pitch_, yaw_);

        // Initialize previous time
        last_time_ = this->get_clock()->now();

        // ================================================
        // map -> odom -> odom_kinematics_map -> base_link transformation
        // ================================================
        map_odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_kinematics_map", 10);
        // Setup tf2 buffer and listener using the node's clock.
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

        // Publisher for the integrated transform
        map_kinematics_transform_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
            "odom_kinematics_tf", 10);
    
        // Timer callback: we integrate periodically (e.g. every 50ms)
        map_kinematics_timer_ = this->create_wall_timer(
            50ms, std::bind(&OdomPublisherNode::publishMapKinematics, this));
    
        // Set the initial time stamp for integration
        map_kinematics_last_update_time_ = this->now();
    
        // Initialize the integrated transform to identity.
        current_odom_kinematics_map_.header.frame_id = "odom_kinematics_map";
        current_odom_kinematics_map_.child_frame_id = base_frame_name_;
        current_odom_kinematics_map_.transform.translation.x = 0.0;
        current_odom_kinematics_map_.transform.translation.y = 0.0;
        current_odom_kinematics_map_.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        current_odom_kinematics_map_.transform.rotation = tf2::toMsg(q);
    }

private:
    void twist_callback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
    {
        latest_twist_ = *msg;
        // if (first_time_)
        // {
        //     return;
        // }
        // Get the current time
        rclcpp::Time current_time = this->get_clock()->now();

        // Compute the time difference
        double dt = (current_time - last_time_).seconds();

        // Extract linear and angular velocities
        double vx_body = msg->twist.twist.linear.x;
        double vy_body = msg->twist.twist.linear.y;
        double vz_body = msg->twist.twist.linear.z;
        double omega_x = msg->twist.twist.angular.x;
        double omega_y = msg->twist.twist.angular.y;
        double omega_z = msg->twist.twist.angular.z;

        // convert angular velocities to radians
        // omega_x = omega_x * M_PI / 180.0;
        // omega_y = omega_y * M_PI / 180.0;
        // omega_z = omega_z * M_PI / 180.0;

        // Get current translation and rotation matrix from SE3 pose (world frame to body frame)
        Eigen::Matrix3d R_world_to_body = pose_.rotation();  // Rotation part of SE3 pose
        Eigen::Vector3d translation_world_to_body = pose_.translation();  // Translation part of SE3 pose

        // Construct the adjoint transformation matrix Ad(T)
        Eigen::MatrixXd adj_T(6, 6);  // Adjoint transformation matrix (6x6)
        adj_T.setZero();  // Initialize to zero
        adj_T.block<3, 3>(0, 0) = R_world_to_body;  // Top left: Rotation matrix
        adj_T.block<3, 3>(3, 3) = R_world_to_body;  // Bottom right: Rotation matrix
        // adj_T.block<3, 3>(3, 0) = skewSymmetricMatrix(translation_world_to_body) * R_world_to_body.transpose();  // Bottom left: Skew-symmetric part
        adj_T.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();

        // Extract linear and angular velocities from the twist message (in body frame)
        Eigen::Vector3d v_linear_body(vx_body, vy_body, vz_body);
        Eigen::Vector3d v_angular_body(omega_x, omega_y, omega_z);

        // Form the velocity twist in the body frame
        Eigen::VectorXd v_body(6);  // 6D velocity twist (linear + angular)
        v_body.head<3>() = v_linear_body;   // Linear part
        v_body.tail<3>() = v_angular_body;  // Angular part

        // Transform the velocity twist to the world frame using the adjoint matrix
        Eigen::VectorXd v_world = adj_T * v_body;

        // Separate the transformed linear and angular velocities
        Eigen::Vector3d v_linear_world = v_world.head<3>();
        Eigen::Vector3d v_angular_world = v_world.tail<3>();

        double vx_world = v_linear_world(0);
        double vy_world = v_linear_world(1);
        double vz_world = v_linear_world(2);
        double omega_x_world = v_angular_world(0);
        double omega_y_world = v_angular_world(1);
        double omega_z_world = v_angular_world(2);

        // Create angular and linear velocities as Eigen vectors
        // Eigen::Vector3d angular_velocity(omega_x_world, omega_y_world, omega_z_world);
        // Eigen::Vector3d angular_velocity(omega_x, omega_y, omega_z);
        Eigen::Vector3d angular_velocity(omega_x, omega_y, omega_z);
        // Eigen::Vector3d linear_velocity(vx_world, vy_world, vz_world);
        Eigen::Vector3d linear_velocity(vx_body, vy_body, vz_body);

        // Create Motion object (angular, linear)
        // pinocchio::Motion v_se3(angular_velocity, linear_velocity);
        pinocchio::Motion v_se3(linear_velocity, angular_velocity);

        // if (first_time_) 
        // {
        //     // tf2::Quaternion tf_q;
        //     // tf_q.setRPY(roll_, pitch_, yaw_);  // Assumes roll, pitch, yaw are in radians
        //     // publish_odometry(x_, y_, z_, tf_q, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, this->get_clock()->now());
        //     // last_time_ = this->get_clock()->now();
        //     v_se3_next_ = v_se3;
        //     first_time_ = false;
        //     return;
        // }
        
        // // Experimental RK4
        // // k1 = f(pose_n, t_n)
        // pinocchio::Motion k1 = v_se3;
        // Eigen::Vector3d prev_lin_vel = v_se3.linear();
        // Eigen::Vector3d prev_ang_vel = v_se3.angular();

        // // k2
        // Eigen::Vector3d angular_velocity_k2 = prev_ang_vel + 0.5 * (prev_ang_vel - angular_velocity);
        // Eigen::Vector3d linear_velocity_k2 = prev_lin_vel + 0.5 * (prev_lin_vel - linear_velocity);
        // pinocchio::Motion v_se3_k2(linear_velocity_k2, angular_velocity_k2);
        // pinocchio::Motion k2 = v_se3_k2;

        // // k3
        // pinocchio::Motion k3 = v_se3_k2;

        // // k4
        // pinocchio::Motion k4 = v_se3_next_;

        // // Compute the average twist
        // pinocchio::Motion twist_avg = (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;

        // // Compute the incremental transformation
        // pinocchio::SE3 delta_SE3 = pinocchio::exp6(twist_avg * dt);



        // Compute the incremental transformation
        pinocchio::SE3 delta_SE3 = pinocchio::exp6(v_se3 * dt);
        // pinocchio::SE3 delta_SE3 = pinocchio::exp6(v_se3_next_ * dt);
        // v_se3_next_ = v_se3;


        // Update the robot's pose by composing with the incremental transformation
        pose_ = pose_.act(delta_SE3);
        // pose_ = integrateRK4(pose_, v_se3, dt);
        // pose_ = pose_ * delta_SE3;
        // pose_ = delta_SE3 * pose_;

        // std::cout << "pose: " << pose_ << std::endl;

        // Extract translation and rotation from the updated pose
        Eigen::Vector3d translation = pose_.translation();
        Eigen::Matrix3d rotation_matrix = pose_.rotation();

        // Convert rotation matrix to Eigen quaternion
        Eigen::Quaterniond eigen_q(rotation_matrix);

        // Convert Eigen quaternion to tf2 quaternion
        tf2::Quaternion q(eigen_q.x(), eigen_q.y(), eigen_q.z(), eigen_q.w());

        // Update position variables
        x_ = translation(0);
        y_ = translation(1);
        z_ = translation(2);

        // Fill in the odometry message
        odom_msg_.header.stamp = msg->header.stamp;

        // Position
        odom_msg_.pose.pose.position.x = x_;
        odom_msg_.pose.pose.position.y = y_;
        odom_msg_.pose.pose.position.z = z_;

        // Orientation (as quaternion)
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
        if (robot == "silver_badger" || robot == "honey_badger")
        {
            odom_to_base_link.child_frame_id = "base_link";
        }
        else
        {
            odom_to_base_link.child_frame_id = "base";
        }

        odom_to_base_link.transform.translation.x = x_;
        odom_to_base_link.transform.translation.y = y_;
        odom_to_base_link.transform.translation.z = z_;
        odom_to_base_link.transform.rotation = tf2::toMsg(q);

        // tf_broadcaster_->sendTransform(odom_to_base_link);

        // Update the last time
        last_time_ = current_time;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Update roll, pitch, and yaw based on the IMU data
        // roll_ = msg->orientation.x;
        // pitch_ = msg->orientation.y;
        // yaw_ = msg->orientation.z;
    }

    void publish_odometry(double x, double y, double z, tf2::Quaternion q, double vx_body, double vy_body, double vz_body, double omega_x, double omega_y, double omega_z, rclcpp::Time stamp)
    {
        // Fill in the odometry message
        odom_msg_.header.stamp = stamp;

        // Position
        odom_msg_.pose.pose.position.x = x;
        odom_msg_.pose.pose.position.y = y;
        odom_msg_.pose.pose.position.z = z;

        // Orientation (as quaternion)
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
        odom_to_base_link.header.stamp = stamp;
        odom_to_base_link.header.frame_id = "odom_kinematics";
        if (robot == "silver_badger" || robot == "honey_badger")
        {
            odom_to_base_link.child_frame_id = "base_link";
        }
        else
        {
            odom_to_base_link.child_frame_id = "base";
        }

        odom_to_base_link.transform.translation.x = x;
        odom_to_base_link.transform.translation.y = y;
        odom_to_base_link.transform.translation.z = z;
        odom_to_base_link.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(odom_to_base_link);
    }

    Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d skew;
        skew <<     0, -v.z(),  v.y(),
                v.z(),     0, -v.x(),
               -v.y(),  v.x(),     0;
        return skew;
    }

    pinocchio::SE3 integrateRK4(const pinocchio::SE3& pose, const pinocchio::Motion& v_se3, double dt)
    {
        // // Step 1: Compute k1 (initial increment)
        // pinocchio::Motion k1 = v_se3;

        // // Step 2: Compute k2 (midpoint increment with k1/2)
        // pinocchio::SE3 delta_k1 = pinocchio::exp6(k1 * (dt / 2.0));  // Half-step transformation
        // pinocchio::Motion v2 = pose.act(delta_k1).actInv(pose) * v_se3;  // Act the motion on delta_k1
        // pinocchio::Motion k2 = v2;

        // // Step 3: Compute k3 (midpoint increment with k2/2)
        // pinocchio::SE3 delta_k2 = pinocchio::exp6(k2 * (dt / 2.0));
        // pinocchio::Motion v3 = pose.act(delta_k2).actInv(pose) * v_se3;
        // pinocchio::Motion k3 = v3;

        // // Step 4: Compute k4 (full-step increment with k3)
        // pinocchio::SE3 delta_k3 = pinocchio::exp6(k3 * dt);  // Full-step transformation
        // pinocchio::Motion v4 = pose.act(delta_k3).actInv(pose) * v_se3;
        // pinocchio::Motion k4 = v4;

        // // Combine the increments using RK4 formula
        // pinocchio::Motion v_final = (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;

        // // Return the updated pose
        // return pose * pinocchio::exp6(v_final * dt);
        // Since the twist is constant, all k_i are equal
        pinocchio::Motion k1 = v_se3;
        pinocchio::Motion k2 = v_se3;
        pinocchio::Motion k3 = v_se3;
        pinocchio::Motion k4 = v_se3;

        // Compute the weighted average of the increments
        pinocchio::Motion v_average = (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;

        // Update the pose
        return pose * pinocchio::exp6(v_average * dt);
    }

    void odom_gt_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!first_time_)
        {
            return;
        }
        first_time_ = false;
        // Update the ground truth position and orientation
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        z_ = msg->pose.pose.position.z;

        // Extract the orientation quaternion
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);

        // Convert the quaternion to roll, pitch, yaw
        tf2::Matrix3x3 m(q);
        m.getRPY(roll_, pitch_, yaw_);

        // roll_ = 0.0;
        // pitch_ = 0.0;
        // yaw_ = 0.0;
        // roll_ =  0.036;
        // pitch_ = -1.604;
        // yaw_ =   1.596;

        double deg_to_rad = M_PI / 180.0;
        roll_ *= deg_to_rad;
        pitch_ *= deg_to_rad;
        yaw_ *= deg_to_rad;

        // Update pose and orientation matrix
        // Create translation vector
        Eigen::Vector3d initial_translation(x_, y_, z_);

        tf2::Quaternion tf_q;
        tf_q.setRPY(roll_, pitch_, yaw_);  // Assumes roll, pitch, yaw are in radians

        publish_odometry(x_, y_, z_, tf_q, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, this->get_clock()->now());

        // Convert tf2 quaternion to Eigen quaternion
        Eigen::Quaterniond eigen_q(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());  // Note the order

        // Convert Eigen quaternion to rotation matrix
        Eigen::Matrix3d initial_rotation = eigen_q.toRotationMatrix();

        // Initialize pose_ as an SE3 object
        pose_ = pinocchio::SE3(initial_rotation, initial_translation);

        

        // Initialize previous time
        last_time_ = this->get_clock()->now();
    }

    // This function now uses Pinocchio to integrate the transform in 3D.
    void publishMapKinematics()
    {
        // Get time difference for integration
        auto current_time = this->now();
        double dt = (current_time - map_kinematics_last_update_time_).seconds();
        map_kinematics_last_update_time_ = current_time;

        // Try to look up the transform from "map" to base_frame_name_
        // and combine it with "odom" -> "map" to get the latest measurement.
        try {
            // Check if there is a new transform available.
            geometry_msgs::msg::TransformStamped map_to_base =
                tf_buffer_->lookupTransform("map", base_frame_name_, tf2::TimePointZero);

            if (map_to_base.header.stamp != last_received_map_base_transform_stamp_)
            {
                // RCLCPP_INFO(this->get_logger(), "New transform available, resetting integration.");
                // New transform available, reset the integration.
                geometry_msgs::msg::TransformStamped odom_to_map =
                    tf_buffer_->lookupTransform("odom", "map", tf2::TimePointZero);
    
                geometry_msgs::msg::TransformStamped odom_to_base =
                    multiplyTransforms(odom_to_map, map_to_base);
    
                // Reset our integrated transform with the new value.
                current_odom_kinematics_map_ = odom_to_base;
                current_odom_kinematics_map_.header.frame_id = "odom_kinematics_map";
                current_odom_kinematics_map_.child_frame_id = base_frame_name_;

                // Update the last received transform stamp
                last_received_map_base_transform_stamp_ = map_to_base.header.stamp;
            }
            else
            {
                // RCLCPP_INFO(this->get_logger(), "No new transform available, performing integration.");
                // No new transform, perform integration
                // Convert current_odom_kinematics_map_ into a Pinocchio SE3 object.
                Eigen::Vector3d t(current_odom_kinematics_map_.transform.translation.x,
                    current_odom_kinematics_map_.transform.translation.y,
                    current_odom_kinematics_map_.transform.translation.z);
                tf2::Quaternion tf_q;
                tf2::fromMsg(current_odom_kinematics_map_.transform.rotation, tf_q);
                Eigen::Quaterniond eigen_q(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());
                Eigen::Matrix3d R = eigen_q.toRotationMatrix();
                pinocchio::SE3 current_pose(R, t);

                // Build the 6D velocity twist from the latest twist (assumed expressed in the base_link frame)
                Eigen::Vector3d linear_vel(latest_twist_.twist.twist.linear.x,
                                           latest_twist_.twist.twist.linear.y,
                                           latest_twist_.twist.twist.linear.z);
                Eigen::Vector3d angular_vel(latest_twist_.twist.twist.angular.x,
                                            latest_twist_.twist.twist.angular.y,
                                            latest_twist_.twist.twist.angular.z);
                pinocchio::Motion v_se3(linear_vel, angular_vel);

                // Compute the incremental transformation using Pinocchio's exponential map
                pinocchio::SE3 delta_SE3 = pinocchio::exp6(v_se3 * dt);

                // Integrate the current pose with the incremental transformation
                current_pose = current_pose.act(delta_SE3);

                // Update the integrated transform message with the new pose
                Eigen::Vector3d new_translation = current_pose.translation();
                Eigen::Matrix3d new_rotation = current_pose.rotation();
                Eigen::Quaterniond new_quat(new_rotation);
                tf2::Quaternion new_tf_q;
                new_tf_q.setX(new_quat.x());
                new_tf_q.setY(new_quat.y());
                new_tf_q.setZ(new_quat.z());
                new_tf_q.setW(new_quat.w());

                current_odom_kinematics_map_.header.stamp = latest_twist_.header.stamp;
                current_odom_kinematics_map_.transform.translation.x = new_translation.x();
                current_odom_kinematics_map_.transform.translation.y = new_translation.y();
                current_odom_kinematics_map_.transform.translation.z = new_translation.z();
                current_odom_kinematics_map_.transform.rotation = tf2::toMsg(new_tf_q);
            }

        } catch (tf2::TransformException &ex) {
            // RCLCPP_WARN(this->get_logger(), "Transform lookup failed, broadcasting zero transform: %s", ex.what());
        
            // It could be that map -> base_link is not available yet, so we broadcast a zero transform.
            current_odom_kinematics_map_.header.stamp = latest_twist_.header.stamp;
            tf_broadcaster_->sendTransform(current_odom_kinematics_map_);
        }

        // Broadcast the transform, it could be integrated or a newly received one
        tf_broadcaster_->sendTransform(current_odom_kinematics_map_);

        // Publish the odometry message (for debugging)
        // Now create an odometry message for debugging
        nav_msgs::msg::Odometry map_odom_msg;
        map_odom_msg.header = current_odom_kinematics_map_.header;  // re-use the same header (with timestamp from the twist message)
        map_odom_msg.header.frame_id = "odom_kinematics_map";         // set as needed
        map_odom_msg.child_frame_id = base_frame_name_;                    // or your base frame

        // Fill in the pose from your integrated transform
        map_odom_msg.pose.pose.position.x = current_odom_kinematics_map_.transform.translation.x;
        map_odom_msg.pose.pose.position.y = current_odom_kinematics_map_.transform.translation.y;
        map_odom_msg.pose.pose.position.z = current_odom_kinematics_map_.transform.translation.z;
        map_odom_msg.pose.pose.orientation = current_odom_kinematics_map_.transform.rotation;
        map_odom_publisher_->publish(map_odom_msg);
        
    }

    // Helper function to multiply two transforms (i.e. perform transformation composition)
    geometry_msgs::msg::TransformStamped multiplyTransforms(
        const geometry_msgs::msg::TransformStamped & A,
        const geometry_msgs::msg::TransformStamped & B)
    {
        tf2::Transform tf_A, tf_B;
        tf2::fromMsg(A.transform, tf_A);
        tf2::fromMsg(B.transform, tf_B);
        tf2::Transform tf_result = tf_A * tf_B;

        geometry_msgs::msg::TransformStamped result;
        result.header.stamp = A.header.stamp;  // Use A’s timestamp (update as needed)
        result.header.frame_id = A.header.frame_id;
        result.child_frame_id = B.child_frame_id;
        result.transform = tf2::toMsg(tf_result);
        return result;
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_odom_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Odometry odom_msg_;

    std::string robot;

    // Odometry state variables
    double x_;
    double y_;
    double z_;
    double roll_;
    double pitch_;
    double yaw_;
    tf2::Matrix3x3 prev_rot_mat_;
    tf2::Matrix3x3 orientation_matrix_;
    pinocchio::SE3 pose_;
    rclcpp::Time last_time_;
    bool first_time_;

    pinocchio::Motion v_se3_next_;

    std::string base_frame_name_;

    // ================================================
    // map -> odom -> odom_kinematics_map -> base_link transformation
    // ================================================
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_odom_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr map_kinematics_transform_pub_;
    rclcpp::TimerBase::SharedPtr map_kinematics_timer_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // The latest transform stamp to check if a new transform has been broadcast
    rclcpp::Time last_received_map_base_transform_stamp_;
    
    // Latest twist (velocity) message
    geometry_msgs::msg::TwistWithCovarianceStamped latest_twist_;
    // For integration time delta
    rclcpp::Time map_kinematics_last_update_time_;
    // The integrated transform from "odom_kinematics_map" to "base_link"   
    geometry_msgs::msg::TransformStamped current_odom_kinematics_map_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
