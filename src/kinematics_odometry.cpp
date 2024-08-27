#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>

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

using namespace std::chrono_literals;



KinematicsOdometry::KinematicsOdometry() : Node("kinematics_odometry")
{
    // QOS to keep most recent messages
    rclcpp::QoS qos_most_recent(rclcpp::KeepLast(1));
    rclcpp::QoS qos_contacts(rclcpp::KeepLast(5));

    feet_contact_sub_ = this->create_subscription<gazebo_msgs::msg::ContactsState>(
        "/feet_contact", qos_contacts, std::bind(&KinematicsOdometry::feetContactCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", qos_most_recent, std::bind(&KinematicsOdometry::jointStateCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/out", qos_most_recent, std::bind(&KinematicsOdometry::imuCallback, this, std::placeholders::_1));

    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/base_lin_vel", 10);
    timer_ = this->create_wall_timer(5ms, std::bind(&KinematicsOdometry::publishVelocity, this));

    std::string urdf = this->declare_parameter("urdf", "");

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

    // Initialize joint states (we don't care about the first "universe" joint)
    pinocchio_joint_names_ = model_.names;
    joint_states_.resize(model_.njoints-1, 0.0);
    joint_states_names_.resize(0);
    joint_velocities_.resize(model_.njoints-1, 0.0);

    // Initialize pinocchio joint map
    for (int i = 0; i < model_.njoints; i++)
    {
        pinocchio_joint_map_[model_.names[i]] = i;
    }

    // Initialize IMU data
    imu_ang_vel_.resize(3, 0.0);
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
    std::string contact_name = msg->states[msg->states.size()-1].collision1_name;
    std::string foot_in_contact_name;

    if (contact_name.find("fl_foot") != std::string::npos)
    {
        foot_in_contact_name = "fl_foot";
    }
    else if (contact_name.find("fr_foot") != std::string::npos)
    {
        foot_in_contact_name = "fr_foot";
    }
    else if (contact_name.find("rl_foot") != std::string::npos)
    {
        foot_in_contact_name = "rl_foot";
    }
    else if (contact_name.find("rr_foot") != std::string::npos)
    {
        foot_in_contact_name = "rr_foot";
    }

    _computeVelocity(foot_in_contact_name);    

}

void KinematicsOdometry::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    joint_states_ = msg->position;
    joint_states_names_ = msg->name;
    joint_velocities_ = msg->velocity;
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
    imu_ang_vel_[0] = msg->angular_velocity.x;
    imu_ang_vel_[1] = msg->angular_velocity.y;
    imu_ang_vel_[2] = msg->angular_velocity.z;
}

void KinematicsOdometry::_computeVelocity(std::string foot_in_contact_name)
{
    // Update forward kinematics for current joint configuration
    Eigen::VectorXd q = Eigen::VectorXd::Zero(joint_states_.size());

    // Map the joint states to the pinocchio joint order
    for (int i=0; i<joint_states_names_.size(); i++)
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
    // pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL, J);
    pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
    // pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::WORLD, J);

    // Get the kinematic chain for the leg
    std::vector<std::string> kinematic_chain;
    if (foot_in_contact_name == "fl_foot")
    {
        kinematic_chain = _getKinematicChain("fl_foot");
    }
    else if (foot_in_contact_name == "fr_foot")
    {
        kinematic_chain = _getKinematicChain("fr_foot");
    }
    else if (foot_in_contact_name == "rl_foot")
    {
        kinematic_chain = _getKinematicChain("rl_foot");
    }
    else if (foot_in_contact_name == "rr_foot")
    {
        kinematic_chain = _getKinematicChain("rr_foot");
    }

    // Find the relavant columns in the Jacobian
    // The joint index-1 corresponds to the columns in the jacobian
    std::vector<int> joint_indices;
    for (auto joint_name : kinematic_chain)
    {
        joint_indices.push_back(pinocchio_joint_map_[joint_name]-1);
    }

    // Extract the leg Jacobian
    Eigen::MatrixXd J_leg = Eigen::MatrixXd::Zero(6, kinematic_chain.size());
    for (int i=0; i<joint_indices.size(); i++)
    {
        J_leg.col(i) = J.col(joint_indices[i]);
    }

    // Choose the joint velocities corresponding to the leg joints
    Eigen::VectorXd leg_joint_velocities = Eigen::VectorXd::Zero(kinematic_chain.size());
    for (int i=0; i<kinematic_chain.size(); i++)
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
    // Eigen::Vector3d imu_ang_vel = R_base_foot.transpose() * leg_velocity.tail(3);
    // Eigen::Vector3d body_velocity = -leg_velocity.head(3);
    Eigen::Vector3d body_velocity = -leg_velocity.head(3) - imu_ang_vel.cross(p_base_foot);
    // Eigen::Vector3d body_velocity = -(R_base_foot.transpose() * leg_velocity.head(3));
    // Eigen::Vector3d body_velocity = -(R_base_foot.transpose() * leg_velocity.head(3)) - imu_ang_vel.cross(p_base_foot);
    // std::cout << body_velocity << std::endl;

    // Store the velocity in the buffer
    leg_velocities_buffer_[foot_in_contact_name] = body_velocity;
}

void KinematicsOdometry::publishVelocity()
{
    // Return if there is nothing in the buffer or if there are less than 2 feet in contact
    if (leg_velocities_buffer_.empty() || leg_velocities_buffer_.size() < 2)
    {
        return;
    }
    // Go through the buffer and compute the average velocity for publishing
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    int legs_in_contact = 0;
    for (auto it = leg_velocities_buffer_.begin(); it != leg_velocities_buffer_.end(); it++)
    {
        // if (it->first == "fl_foot")
        // {
            velocity += it->second;
            legs_in_contact++;
        // }
    }
    velocity /= legs_in_contact;

    // std::cout << "Velocity: " << std::endl;
    // std::cout << velocity.transpose() << std::endl;
    // std::cout << "Feet in contact: " << legs_in_contact << std::endl;

    // Form ros message
    geometry_msgs::msg::TwistStamped velocity_msg;
    velocity_msg.header.stamp = this->get_clock()->now();
    velocity_msg.header.frame_id = "base_link";
    velocity_msg.twist.linear.x = velocity(0);
    velocity_msg.twist.linear.y = velocity(1);
    velocity_msg.twist.linear.z = velocity(2);

    // velocity_msg.twist.angular.x = 0;
    // velocity_msg.twist.angular.y = 0;
    // velocity_msg.twist.angular.z = 0;

    velocity_msg.twist.angular.x = imu_ang_vel_[0];
    velocity_msg.twist.angular.y = imu_ang_vel_[1];
    velocity_msg.twist.angular.z = imu_ang_vel_[2];

    velocity_pub_->publish(velocity_msg);

    // Clear the buffer
    leg_velocities_buffer_.clear();
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicsOdometry>());
  rclcpp::shutdown();
  return 0;
}