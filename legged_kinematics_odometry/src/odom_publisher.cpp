#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/imu.hpp>

#include <pinocchio/spatial/se3.hpp>
#include "pinocchio/spatial/explog.hpp"
#include <pinocchio/spatial/motion.hpp>
#include <Eigen/Geometry>
#include <cmath>


class OdomPublisherNode : public rclcpp::Node
{
public:
    OdomPublisherNode()
        : Node("odom_publisher_node")
    {
        // Initialize subscriber to "base_lin_vel" topic
        twist_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
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
            odom_msg_.child_frame_id = "base_link";  // Robot base frame
        }
        else
        {
            odom_msg_.child_frame_id = "base";  // Robot base frame
        }

        // Initialize position and orientation
        first_time_ = true;
        x_ = -2.0497805745819044;
        y_ = 3.501646823445426;
        z_ = 0.2930235459349441;
        roll_ =  -0.0008285592586317505;
        pitch_ = -0.035986024291907226;
        yaw_ =   0.0069288823779056225;
        
        // x_ = -2.0459031821872986;
        // y_ = 3.496181887042353;
        // z_ = 0.3003354309985301;
        // roll_ =  0.036;
        // pitch_ = -1.604;
        // yaw_ =   1.596;
        // x_ = -2.0;
        // y_ = 3.5;
        // z_ = 0.05;
        // x_ = 0.0;
        // y_ = 0.0;
        // z_ = 0.0;
        // roll_ = 0.0;
        // pitch_ = 0.0;
        // yaw_ = 0.0;

        // double deg_to_rad = M_PI / 180.0;
        // roll_ *= deg_to_rad;
        // pitch_ *= deg_to_rad;
        // yaw_ *= deg_to_rad;

        // Create translation vector
        Eigen::Vector3d initial_translation(x_, y_, z_);

        tf2::Quaternion tf_q;
        tf_q.setRPY(roll_, pitch_, yaw_);  // Assumes roll, pitch, yaw are in radians

        publish_odometry(x_, y_, z_, tf_q, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, this->get_clock()->now());

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
    }

private:
    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        // if (first_time_)
        // {
        //     return;
        // }
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

        tf_broadcaster_->sendTransform(odom_to_base_link);

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

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscriber_;
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
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
