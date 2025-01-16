#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <gazebo/common/Plugin.hh>
#include "gazebo/common/PID.hh"
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <hb40_commons/msg/joint_command.hpp>
#include <hb40_commons/msg/robot_state.hpp>
#include <hb40_commons/msg/leg_state.hpp>

// include files for contact sensor
#include <thread>
#include <unordered_map>

using namespace std::chrono_literals;

namespace gazebo
{
    class UnitreeGazeboControlPlugin : public ModelPlugin
    {
    public:
        UnitreeGazeboControlPlugin() : node_(std::make_shared<rclcpp::Node>("unitree_gazebo_control_plugin"))
        {
            prev_errors_.resize(0);
        }

        void Load(physics::ModelPtr model, sdf::ElementPtr sdf)
        {
            RCLCPP_INFO(rclcpp::get_logger("unitree_gazebo_control_plugin"), "Loading unitree_gazebo_ros2_control plugin");
            model_ = model;

            // Check which robot model is being used
            robot = sdf->Get<std::string>("robot");
            RCLCPP_INFO(node_->get_logger(), "Gazebo Control for Robot model: %s", robot.c_str());

            // Set the joint topic based on the robot model
            if (robot == "a1")
            {
                joint_topic_ = "/a1/joint_commandHighPrio";
            }
            else if (robot == "go1")
            {
                joint_topic_ = "/go1/joint_commandHighPrio";
            }
            else if (robot == "go2")
            {
                joint_topic_ = "/go2/joint_commandHighPrio";
            }
            

            // Get joints from the model
            joints_ = model_->GetJoints();

            // Initialize previous errors vector
            prev_errors_.resize(joints_.size(), 0.0);

            // QoS
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
            qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

            // Contact sensor QoS
            auto qos_contacts = rclcpp::QoS(rclcpp::KeepLast(5));
            qos_contacts.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

            // Contact sensor publisher
            contact_state_publisher_ = node_->create_publisher<hb40_commons::msg::RobotState>("/feet_contact_state", 10);

            // Get sensor manager
            sensors_ = sensors::SensorManager::Instance();

            // Read contact sensor data with sensor manager
            contact_sensors_["fr_foot"] = std::dynamic_pointer_cast<sensors::ContactSensor>(sensors_->GetSensor("fr_foot_bumper"));
            contact_sensors_["fl_foot"] = std::dynamic_pointer_cast<sensors::ContactSensor>(sensors_->GetSensor("fl_foot_bumper"));
            contact_sensors_["rr_foot"] = std::dynamic_pointer_cast<sensors::ContactSensor>(sensors_->GetSensor("rr_foot_bumper"));
            contact_sensors_["rl_foot"] = std::dynamic_pointer_cast<sensors::ContactSensor>(sensors_->GetSensor("rl_foot_bumper"));

            // Create a timer to publish contact states
            auto publish_frequency = std::chrono::milliseconds(10);
            contact_timer_ = node_->create_wall_timer(publish_frequency, std::bind(&UnitreeGazeboControlPlugin::publishContactStates, this));

            // rclcpp::NodeOptions options;
            // options.parameter_overrides({{"use_sim_time", true}});
            // this->node_ = std::make_shared<rclcpp::Node>("mab_gazebo_control_plugin", options);

            // Subscribe to the joint command topic
            joint_command_sub_ = node_->create_subscription<hb40_commons::msg::JointCommand>(
                joint_topic_, qos, std::bind(&UnitreeGazeboControlPlugin::OnJointCommand, this, std::placeholders::_1));

            // Joint state publisher
            joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
            timer_ = node_->create_wall_timer(
                10ms, std::bind(&UnitreeGazeboControlPlugin::publishJointStates, this));

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
                    // if (robot == "honey_badger" && joint_name == "sp_j0")
                    // {
                    //     joint_state_msg.position.push_back(0.0);
                    //     joint_state_msg.velocity.push_back(0.0);
                    //     joint_state_msg.effort.push_back(0.0);
                    //     break;
                    // }
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
            // if (msg->t_pos.size() != joints_.size() || msg->kp.size() != joints_.size() || msg->kd.size() != joints_.size())
            // {
            //     RCLCPP_ERROR(node_->get_logger(), "Size of t_pos, kp, and kd must match the number of joints");
            //     return;
            // }

            // Assuming a control loop period of 50Hz (as in mab_locomotion policy node)
            // double dt = 0.02;

            for (size_t i = 0; i < msg->t_pos.size(); ++i)
            {
                // double position_error = msg->t_pos[i] - joints_[i]->Position(0);
                // double derivative_error = (position_error - prev_errors_[i]) / dt;

                // double torque = msg->kp[i] * position_error + msg->kd[i] * derivative_error;

                // Find the correct joint to apply the torque
                std::string joint_name;
                joint_name = joint_names_input_[i];

                for (size_t j = 0; j < joints_.size(); ++j)
                {
                    // RCLCPP_INFO(node_->get_logger(), "Joint %s: ", joints_[j]->GetName().c_str());
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

        void publishContactStates()
        {
            // Get the contact state
            hb40_commons::msg::RobotState robot_state;
            gazebo::common::Time sim_time = model_->GetWorld()->SimTime();
            robot_state.header.stamp.sec = sim_time.sec;
            robot_state.header.stamp.nanosec = sim_time.nsec;
            robot_state.header.frame_id = "base_link";
            robot_state.leg.resize(4);
            robot_state.leg.clear();

            // Get the contact state for each foot
            for (auto &contact_sensor : contact_sensors_)
            {
                std::string leg_name = contact_sensor.first;
                sensors::ContactSensorPtr contact_sensor_ptr = contact_sensor.second;

                // Check if the foot is in contact
                bool is_contact = detectContact(contact_sensor_ptr);
                hb40_commons::msg::LegState leg_state;
                leg_state.leg_name = leg_name;
                leg_state.contact = is_contact;

                // Retrieve the contact force
                if (is_contact)
                {
                    physics::Contact contact = contact_sensor_ptr->Contacts(contact_sensor_ptr->GetCollisionName(0)).begin()->second;
                    ignition::math::Vector3d force = contact.wrench[0].body1Force;
                    leg_state.foot_force_est.x = force.X();
                    leg_state.foot_force_est.y = force.Y();
                    leg_state.foot_force_est.z = force.Z();
                }
                else
                {
                    leg_state.foot_force_est.x = 0.0;
                    leg_state.foot_force_est.y = 0.0;
                    leg_state.foot_force_est.z = 0.0;
                }

                robot_state.leg.push_back(leg_state);
            }

            // Publish the contact state
            contact_state_publisher_->publish(robot_state);
        }

        bool detectContact(sensors::ContactSensorPtr contact_sensor)
        {
            for (unsigned int i = 0; i < contact_sensor->GetCollisionCount(); i++)
            {
                std::string collision_name = contact_sensor->GetCollisionName(i);
                std::map<std::string, physics::Contact> contacts = contact_sensor->Contacts(collision_name);
                if (contacts.size() > 0)
                {
                    return true;
                }
            }
            return false;
        }

    private:
        physics::ModelPtr model_;
        std::vector<physics::JointPtr> joints_;
        std::vector<double> prev_errors_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<hb40_commons::msg::JointCommand>::SharedPtr joint_command_sub_;
        std::string joint_topic_;
        std::thread ros_thread_;

        // Contact sensor
        rclcpp::Publisher<hb40_commons::msg::RobotState>::SharedPtr contact_state_publisher_;
        std::unordered_map<std::string, sensors::ContactSensorPtr> contact_sensors_;
        sensors::SensorManager *sensors_;
        rclcpp::TimerBase::SharedPtr contact_timer_;

        // Joint state publisher
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        // List of joints in the order that they are received from the controller
        std::string robot;
        std::vector<std::string> joint_names_input_ = {
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint"};
    };

    GZ_REGISTER_MODEL_PLUGIN(UnitreeGazeboControlPlugin)
}
