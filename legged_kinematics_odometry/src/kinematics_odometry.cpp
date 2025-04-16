#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <iomanip>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <legged-kinematics-odometry/kinematics_odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hb40_commons/msg/robot_state.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;


// Implementing the EMAFilter constructor
EMAFilter::EMAFilter(double alpha)
    : alpha_(alpha), initialized_(false) {}

// Implementing the filter function
Eigen::Vector3d EMAFilter::filter(const Eigen::Vector3d &new_value)
{
    if (!initialized_)
    {
        filtered_value_ = new_value;
        initialized_ = true;
    }
    else
    {
        filtered_value_ = alpha_ * new_value + (1.0 - alpha_) * filtered_value_;
    }
    return filtered_value_;
}




KinematicsOdometry::KinematicsOdometry() : Node("kinematics_odometry")
{
    // QOS to keep most recent messages
    rclcpp::QoS qos_most_recent(rclcpp::KeepLast(1));
    rclcpp::QoS qos_contacts(rclcpp::KeepLast(5));

    std::string urdf = this->declare_parameter("urdf", "");
    real_robot_ = this->declare_parameter("real_robot", false);

    std::string feet_contact_topic;
    if (real_robot_)
    {
        feet_contact_topic = "/estimated_contact_state";
    }
    else
    {
        feet_contact_topic = "/feet_contact_state";
    }
    feet_contact_state_sub_ = this->create_subscription<hb40_commons::msg::RobotState>(
        feet_contact_topic, qos_contacts, std::bind(&KinematicsOdometry::feetContactStateCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", qos_most_recent, std::bind(&KinematicsOdometry::jointStateCallback, this, std::placeholders::_1));
        // "/joint_states_filtered", qos_most_recent, std::bind(&KinematicsOdometry::jointStateCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_gt_fd", qos_most_recent, std::bind(&KinematicsOdometry::imuCallback, this, std::placeholders::_1));
    // "/imu/out", qos_most_recent, std::bind(&KinematicsOdometry::imuCallback, this, std::placeholders::_1));

    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/base_lin_vel", 10);
    timer_ = this->create_wall_timer(5ms, std::bind(&KinematicsOdometry::publishVelocity, this));

    

    if (urdf.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No URDF file specified");
        return;
    }

    // Load the URDF file
    pinocchio::urdf::buildModel(urdf, model_);

    // Create data structure
    data_ = pinocchio::Data(model_);

    Eigen::VectorXd q;

    // Print the model details
    std::cout << "URDF model " << model_.name << " has " << model_.njoints << " joints." << std::endl;
    std::cout << "Dimension of velocity vector space: " << model_.nv << std::endl;
    std::cout << "Degrees of freedom: " << model_.nq << std::endl;

    for (int i = 0; i < model_.njoints; i++)
    {
        std::cout << "Joint " << i << " has name " << model_.names[i] << std::endl;
    }

    // Initialize covariance matrix depending on the model
    Eigen::Matrix3d linear_covariance;
    linear_covariance << 0.1, 0.0,       0.0,
                         0.0,        0.1, 0.0,
                         0.0,        0.0,       0.05;

    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
    covariance.block<3, 3>(0, 0) = linear_covariance;
    covariance.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < 6; i++) {
        for (size_t j = 0; j < 6; j++) {
          covariance_msg[i * 6 + j] = covariance(i, j);  // Row-major order.
        }
    }              

    // Initialize joint states (we don't care about the first "universe" joint)
    pinocchio_joint_names_ = model_.names;
    joint_states_.resize(model_.njoints - 1, 0.0);
    joint_states_names_.resize(0);
    joint_velocities_.resize(model_.njoints - 1, 0.0);

    // Initialize pinocchio joint map
    for (int i = 0; i < model_.njoints; i++)
    {
        pinocchio_joint_map_[model_.names[i]] = i;
    }

    // Initialize foot names
    if (model_.name == "a1" || model_.name == "go1" || model_.name == "go2")
    {
        foot_names_["fl"] = "FL_foot";
        foot_names_["fr"] = "FR_foot";
        foot_names_["rl"] = "RL_foot";
        foot_names_["rr"] = "RR_foot";
    }
    else
    {
        foot_names_["fl"] = "fl_foot";
        foot_names_["fr"] = "fr_foot";
        foot_names_["rl"] = "rl_foot";
        foot_names_["rr"] = "rr_foot";
    }

    // Initialize IMU data
    imu_orientation_.resize(4, 0.0);
    imu_ang_vel_.resize(3, 0.0);

    // Initialize EMAFilter for velocity smoothing
    velocity_filter_ = std::make_shared<EMAFilter>(1.0);
}

void KinematicsOdometry::feetContactCallback(const gazebo_msgs::msg::ContactsState::SharedPtr msg)
{
    // When a contact for a foot is detected, we compute forward kinematics and compute the velocity of the leg
    // The average of all foot velocities is take for the feet on the ground
    if (joint_states_names_.empty() || msg->states.empty())
    {
        return;
    }

    // Contact name
    std::string contact_name = msg->states[msg->states.size() - 1].collision1_name;
    std::string foot_in_contact_name;

    if (contact_name.find("fl_foot") != std::string::npos)
    {
        foot_in_contact_name = foot_names_["fl"];
    }
    else if (contact_name.find("fr_foot") != std::string::npos)
    {
        foot_in_contact_name = foot_names_["fr"];
    }
    else if (contact_name.find("rl_foot") != std::string::npos)
    {
        foot_in_contact_name = foot_names_["rl"];
    }
    else if (contact_name.find("rr_foot") != std::string::npos)
    {
        foot_in_contact_name = foot_names_["rr"];
    }

    _computeLegVelocity(foot_in_contact_name);
}

void KinematicsOdometry::feetContactStateCallback(const hb40_commons::msg::RobotState::SharedPtr msg)
{
    // When a contact for a foot is detected, we compute forward kinematics and compute the velocity of the leg
    // The average of all foot velocities is take for the feet on the ground
    if (joint_states_names_.empty() || msg->leg.empty())
    {
        return;
    }

    // Contact name
    std::string foot_in_contact_name;

    for (auto leg_state : msg->leg)
    {
        if (leg_state.contact_gt)
        {
            foot_in_contact_name = foot_names_[leg_state.leg_name.substr(0, 2)];
            _computeLegVelocity(foot_in_contact_name);
        }
    }
}

void KinematicsOdometry::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    joint_states_ = msg->position;
    joint_states_names_ = msg->name;
    joint_velocities_ = msg->velocity;

    // Store timestamp
    joint_states_timestamp_ = msg->header.stamp;
}

std::vector<std::string> KinematicsOdometry::_getKinematicChain(const std::string &link_name)
{
    pinocchio::FrameIndex frame1_idx = model_.getFrameId("universe");
    pinocchio::FrameIndex frame2_idx = model_.getFrameId(link_name);

    // Get the joint indices corresponding to these frames
    pinocchio::JointIndex joint1_idx = model_.frames[frame1_idx].parent;
    pinocchio::JointIndex joint2_idx = model_.frames[frame2_idx].parent;

    // Trace the kinematic chain from joint2 to joint1
    std::vector<std::string> kinematic_chain;
    pinocchio::JointIndex current_joint_idx = joint2_idx;

    while (current_joint_idx != joint1_idx && current_joint_idx != 0)
    {
        // Add the joint name to the list
        kinematic_chain.push_back(model_.names[current_joint_idx]);

        // Move to the parent joint
        current_joint_idx = model_.parents[current_joint_idx];
    }

    // Reverse the list to go from link1 to link2
    std::reverse(kinematic_chain.begin(), kinematic_chain.end());

    return kinematic_chain;
}

void KinematicsOdometry::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_orientation_[0] = msg->orientation.x;
    imu_orientation_[1] = msg->orientation.y;
    imu_orientation_[2] = msg->orientation.z;
    imu_orientation_[3] = msg->orientation.w;

    imu_ang_vel_[0] = msg->angular_velocity.x;
    imu_ang_vel_[1] = msg->angular_velocity.y;
    imu_ang_vel_[2] = msg->angular_velocity.z;
}

void KinematicsOdometry::_computeLegVelocity(std::string foot_in_contact_name)
{
    // Update forward kinematics for current joint configuration
    Eigen::VectorXd q = Eigen::VectorXd::Zero(joint_states_.size());

    // Map the joint states to the pinocchio joint order
    for (int i = 0; i < joint_states_names_.size(); i++)
    {
        auto it = std::find(pinocchio_joint_names_.begin(), pinocchio_joint_names_.end(), joint_states_names_[i]);
        int idx = std::distance(pinocchio_joint_names_.begin(), it) - 1;
        q(idx) = joint_states_[i];
    }

    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);

    // Find the frame index and compute the Jacobian
    pinocchio::FrameIndex frame_id = model_.getFrameId(foot_in_contact_name);
    Eigen::MatrixXd J(6, model_.nv);
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

    // Get the kinematic chain for the leg
    std::vector<std::string> kinematic_chain;
    if (foot_in_contact_name == foot_names_["fl"])
    {
        kinematic_chain = _getKinematicChain(foot_names_["fl"]);
    }
    else if (foot_in_contact_name == foot_names_["fr"])
    {
        kinematic_chain = _getKinematicChain(foot_names_["fr"]);
    }
    else if (foot_in_contact_name == foot_names_["rl"])
    {
        kinematic_chain = _getKinematicChain(foot_names_["rl"]);
    }
    else if (foot_in_contact_name == foot_names_["rr"])
    {
        kinematic_chain = _getKinematicChain(foot_names_["rr"]);
    }

    // Find the relavant columns in the Jacobian
    // The joint index-1 corresponds to the columns in the jacobian
    std::vector<int> joint_indices;
    for (auto joint_name : kinematic_chain)
    {
        joint_indices.push_back(pinocchio_joint_map_[joint_name] - 1);
    }

    // Extract the leg Jacobian
    Eigen::MatrixXd J_leg = Eigen::MatrixXd::Zero(6, kinematic_chain.size());
    for (int i = 0; i < joint_indices.size(); i++)
    {
        J_leg.col(i) = J.col(joint_indices[i]);
    }

    // Choose the joint velocities corresponding to the leg joints
    Eigen::VectorXd leg_joint_velocities = Eigen::VectorXd::Zero(kinematic_chain.size());
    for (int i = 0; i < kinematic_chain.size(); i++)
    {
        auto it = std::find(joint_states_names_.begin(), joint_states_names_.end(), kinematic_chain[i]);
        int idx = std::distance(joint_states_names_.begin(), it);
        leg_joint_velocities(i) = joint_velocities_[idx];
    }

    // The Jacobian will give us velocities in the foot frame, we need to convert it to the base frame
    // Rotation from base_link to foot
    Eigen::Matrix3d R_base_foot = data_.oMf[frame_id].rotation();

    // Store the translation from base_link to foot, will be used to compute velocity of the body later
    Eigen::Vector3d p_base_foot = data_.oMf[frame_id].translation();

    // // Stacked rotation matrix for rotating the Jacobian
    // Eigen::MatrixXd T_world_end_effector = Eigen::MatrixXd::Zero(6, 6);
    // T_world_end_effector.block<3, 3>(0, 0) = R_base_foot;
    // T_world_end_effector.block<3, 3>(3, 3) = R_base_foot;
    // // T_world_end_effector.block<3, 3>(0, 0) = R_base_foot.transpose();
    // // T_world_end_effector.block<3, 3>(3, 3) = R_base_foot.transpose();

    // Compute the leg velocity
    // Set the z velocity component to zero, we cannot move in the z direction if there is a contact (replace 3rd row with zeros)
    // J_leg.row(2) = Eigen::VectorXd::Zero(kinematic_chain.size());
    Eigen::VectorXd leg_velocity = J_leg * leg_joint_velocities;

    // Eigen::Vector3d imu_ang_vel = leg_velocity.tail(3);
    Eigen::Vector3d imu_ang_vel(imu_ang_vel_[0], imu_ang_vel_[1], imu_ang_vel_[2]);
    Eigen::Vector3d body_velocity_leg = -leg_velocity.head(3) - imu_ang_vel.cross(p_base_foot);

    // Store the velocity in the buffer
    leg_velocities_buffer_[foot_in_contact_name] = body_velocity_leg;

    // Fill buffers for least squares data
    feet_positions_[foot_in_contact_name] = p_base_foot;
    feet_velocities_[foot_in_contact_name] = leg_velocity.head(3);
}

void KinematicsOdometry::_computeBodyVelocity()
{
    bool least_squares = true;
    if (least_squares)
    {

        int nLegs = leg_velocities_buffer_.size();
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6 * leg_velocities_buffer_.size(), 6);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(6 * leg_velocities_buffer_.size());

        int i = 0;
        for (auto it = feet_positions_.begin(); it != feet_positions_.end(); it++)
        {
            // Fill the A matrix
            A.block<3, 3>(6 * i, 0) = Eigen::Matrix3d::Identity();
            A.block<3, 3>(6 * i, 3) = -skewSymmetricMatrix(feet_positions_[it->first]);
            A.block<3, 3>(6 * i + 3, 3) = Eigen::Matrix3d::Identity();

            // Fill the b vector, add imu angular velocity as regularization
            b.segment(6 * i, 3) = -feet_velocities_[it->first];
            b(6 * i + 3) = imu_ang_vel_[0];
            b(6 * i + 4) = imu_ang_vel_[1];
            b(6 * i + 5) = imu_ang_vel_[2];
            i++;
        }

        // Define the square-root weight matrix W_half (block-diagonal)
        // For each leg, the first 3 rows (foot velocity measurement) use a weight of 1 for x, y, and z.
        // The IMU constraints (next 3 rows) are given a lower weight.
        Eigen::MatrixXd W_half = Eigen::MatrixXd::Zero(6 * nLegs, 6 * nLegs);
        // double imu_weight = 1.0;  // Lower weight for the IMU angular velocity regularization
        double imu_weight = 0.8;  // Lower weight for the IMU angular velocity regularization

        for (int i = 0; i < nLegs; i++) {
            // For foot velocity measurement (first 3 rows)
            W_half(6 * i + 0, 6 * i + 0) = 1.0;  // x
            W_half(6 * i + 1, 6 * i + 1) = 1.0;  // y
            W_half(6 * i + 2, 6 * i + 2) = 1.0;  // z

            // For IMU angular velocity regularization (next 3 rows) with less importance
            W_half(6 * i + 3, 6 * i + 3) = imu_weight;
            W_half(6 * i + 4, 6 * i + 4) = imu_weight;
            W_half(6 * i + 5, 6 * i + 5) = imu_weight;
        }

        // Apply the weight matrix to both A and b to form the weighted least squares problem
        Eigen::MatrixXd A_weighted = W_half * A;
        Eigen::VectorXd b_weighted = W_half * b;

        // Solve the weighted least squares problem: A_weighted * x = b_weighted
        Eigen::VectorXd x;
        if (real_robot_)
        {
            x = A_weighted.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_weighted);
            // x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        }
        else
        {
            x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        }
        // Eigen::VectorXd x = A_weighted.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_weighted);

        // Solve the least squares problem Ax = b
        // Eigen::VectorXd x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        body_linear_velocity = x.head(3);
        body_angular_velocity = x.tail(3);

        //Norm of the linear velocity
        // std::cout << "Velocity Norm: " << body_linear_velocity.norm() << std::endl;

        // // Print the rank of A matrix
        // std::cout << "Buffer size" << leg_velocities_buffer_.size() << std::endl;
        // std::cout << "Rank of A: " << A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).rank() << std::endl;

        // // Compare with average velocity and imu angular velocity
        // Eigen::Vector3d avg_velocity = Eigen::Vector3d::Zero();
        // for (auto it = leg_velocities_buffer_.begin(); it != leg_velocities_buffer_.end(); it++)
        // {
        //     avg_velocity += it->second;
        // }
        // avg_velocity /= leg_velocities_buffer_.size();
        // std::cout << "Average linear velocity: " << avg_velocity.transpose() << std::endl;
        // std::cout << "IMU angular velocity: " << imu_ang_vel_[0] << " " << imu_ang_vel_[1] << " " << imu_ang_vel_[2] << std::endl;
        // std::cout << "Least squares linear velocity: " << x.head(3) << std::endl;
        // std::cout << "Least squares angular velocity: " << x.tail(3) << std::endl;
        // std::cout << " ======================== " << std::endl;
    }
    else
    {
        body_linear_velocity = Eigen::Vector3d::Zero();
        // Average the velocities of the legs as the body velocity
        for (auto it = leg_velocities_buffer_.begin(); it != leg_velocities_buffer_.end(); it++)
        {
            body_linear_velocity += it->second;
        }
        body_linear_velocity /= leg_velocities_buffer_.size();

        // Apply EMA filter to the linear velocity before publishing
        body_linear_velocity = velocity_filter_->filter(body_linear_velocity);
        body_angular_velocity[0] = imu_ang_vel_[0];
        body_angular_velocity[1] = imu_ang_vel_[1];
        body_angular_velocity[2] = imu_ang_vel_[2];

        std::cout << "Velocity Norm: " << body_linear_velocity.norm() << std::endl;
    }
}

void KinematicsOdometry::publishVelocity()
{
    // Return if there is nothing in the buffer or if there are less than 2 feet in contact
    if (leg_velocities_buffer_.empty() || leg_velocities_buffer_.size() < 2)
    {
        RCLCPP_WARN(this->get_logger(), "Not enough legs in contact to compute body velocity");
        return;
    }

    _computeBodyVelocity();

    geometry_msgs::msg::TwistWithCovarianceStamped velocity_msg;

    // Publish the velocity
    // Use the joint state message timestamp
    if (real_robot_)
    {
        velocity_msg.header.stamp = joint_states_timestamp_;
    }
    else
    {
        velocity_msg.header.stamp = this->get_clock()->now();
    }
    velocity_msg.header.frame_id = "base_link";
    velocity_msg.twist.twist.linear.x = body_linear_velocity(0);
    velocity_msg.twist.twist.linear.y = body_linear_velocity(1);
    velocity_msg.twist.twist.linear.z = body_linear_velocity(2);

    velocity_msg.twist.twist.angular.x = body_angular_velocity(0);
    velocity_msg.twist.twist.angular.y = body_angular_velocity(1);
    velocity_msg.twist.twist.angular.z = body_angular_velocity(2);

    velocity_msg.twist.covariance = covariance_msg;

    velocity_pub_->publish(velocity_msg);

    // Clear the buffer
    leg_velocities_buffer_.clear();
    feet_positions_.clear();
    feet_velocities_.clear();
}

Eigen::Matrix3d KinematicsOdometry::skewSymmetricMatrix(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d skew;
    skew << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return skew;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsOdometry>());
    rclcpp::shutdown();
    return 0;
}