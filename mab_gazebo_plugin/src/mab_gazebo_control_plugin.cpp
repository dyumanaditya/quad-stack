#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <gazebo/common/Plugin.hh>
#include "gazebo/common/PID.hh"
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>

#include <hb40_commons/msg/joint_command.hpp>

using namespace std::chrono_literals;

namespace gazebo
{
    class MABGazeboControlPlugin : public ModelPlugin
    {
    public:
        MABGazeboControlPlugin() : node_(std::make_shared<rclcpp::Node>("mab_gazebo_control_plugin"))
        {
            prev_errors_.resize(0);
        }

        void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
        {
            RCLCPP_INFO(rclcpp::get_logger("mab_gazebo_control_plugin"), "Loading mab_gazebo_ros2_control plugin");
            model_ = model;
            node_->declare_parameter("joint_topic", "/hb40/joint_commandHighPrio");
            node_->get_parameter("joint_topic", joint_topic_);

            // Get joints from the model
            joints_ = model_->GetJoints();

            // Initialize previous errors vector
            prev_errors_.resize(joints_.size(), 0.0);

            // QoS
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

            // rclcpp::NodeOptions options;
            // options.parameter_overrides({{"use_sim_time", true}});
            // this->node_ = std::make_shared<rclcpp::Node>("mab_gazebo_control_plugin", options);

            // Subscribe to the joint command topic
            joint_command_sub_ = node_->create_subscription<hb40_commons::msg::JointCommand>(
                joint_topic_, qos, std::bind(&MABGazeboControlPlugin::OnJointCommand, this, std::placeholders::_1));

            // Joint state publisher
            joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
            timer_ = node_->create_wall_timer(
                10ms, std::bind(&MABGazeboControlPlugin::publishJointStates, this));

            // Run ROS 2 async spinner in a separate thread
            ros_thread_ = std::thread([this]()
                                      { rclcpp::spin(node_); });
        }

        void publishJointStates()
        {
            sensor_msgs::msg::JointState joint_state_msg;
            gazebo::common::Time sim_time = model_->GetWorld()->SimTime();
            joint_state_msg.header.stamp.sec = sim_time.sec;
            joint_state_msg.header.stamp.nanosec = sim_time.nsec;
            
            joint_state_msg.name = joint_names_input_;

            for (size_t i = 0; i < joint_names_input_.size(); ++i)
            {
                // Find the correct joint
                std::string joint_name = joint_names_input_[i];   
                for (size_t j = 0; j < joints_.size(); ++j)
                {
                    if (joints_[j]->GetName() == joint_name)
                    {
                        joint_state_msg.position.push_back(joints_[j]->Position(0));
                        joint_state_msg.velocity.push_back(joints_[j]->GetVelocity(0));
                        joint_state_msg.effort.push_back(joints_[j]->GetForce(0));
                        break;
                    }
                }
            }
            joint_state_publisher_->publish(joint_state_msg);
        }

        void OnJointCommand(const hb40_commons::msg::JointCommand::SharedPtr msg)
        {
            if (msg->t_pos.size() != joints_.size() || msg->kp.size() != joints_.size() || msg->kd.size() != joints_.size())
            {
                RCLCPP_ERROR(node_->get_logger(), "Size of t_pos, kp, and kd must match the number of joints");
                return;
            }

            // Assuming a control loop period of 50Hz (as in mab_locomotion policy node)
            // double dt = 0.02; 

            for (size_t i = 0; i < msg->t_pos.size(); ++i)
            {
                // double position_error = msg->t_pos[i] - joints_[i]->Position(0);
                // double derivative_error = (position_error - prev_errors_[i]) / dt;

                // double torque = msg->kp[i] * position_error + msg->kd[i] * derivative_error;

                // Find the correct joint to apply the torque
                std::string joint_name = joint_names_input_[i];
                for (size_t j = 0; j < joints_.size(); ++j)
                {
                    if (joints_[j]->GetName() == joint_name)
                    {
                        // joints_[j]->SetForce(0, torque);
                        model_->GetJointController()->SetPositionPID(joints_[j]->GetScopedName(), common::PID(msg->kp[i], 0, msg->kd[i]));
                        // model_->GetJointController()->SetJointPosition(joints_[j]->GetScopedName(), msg->t_pos[i]);
                        model_->GetJointController()->SetPositionTarget(joints_[j]->GetScopedName(), msg->t_pos[i]);
                        // joints_[j]->SetPosition(0, msg->t_pos[i]);
                        // Print the name of the joint
                        // std::string j_name = joints_[j]->GetName();
                        // RCLCPP_INFO(node_->get_logger(), "Joint %s: Applying torque %f", j_name.c_str(), torque);
                        break;
                    }
                }

                // joints_[i]->SetForce(0, torque);

                // Update previous error
                // prev_errors_[i] = position_error;
            }
        }

    private:
        physics::ModelPtr model_;
        std::vector<physics::JointPtr> joints_;
        std::vector<double> prev_errors_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<hb40_commons::msg::JointCommand>::SharedPtr joint_command_sub_;
        std::string joint_topic_;
        std::thread ros_thread_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        // List of joints in the order that they are received from the controller
        std::vector<std::string> joint_names_input_ = {
            "fr_j0",
            "fr_j1",
            "fr_j2",
            "fl_j0",
            "fl_j1",
            "fl_j2",
            "rl_j0",
            "rl_j1",
            "rl_j2",
            "rr_j0",
            "rr_j1",
            "rr_j2",
            "sp_j0"
        };
    };

    GZ_REGISTER_MODEL_PLUGIN(MABGazeboControlPlugin)
}
