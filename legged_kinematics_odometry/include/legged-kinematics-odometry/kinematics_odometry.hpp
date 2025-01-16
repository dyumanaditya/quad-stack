#include <vector>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <gazebo_msgs/msg/contacts_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <hb40_commons/msg/robot_state.hpp>

// EMA Filter class for smoothing velocities
class EMAFilter {
public:
    EMAFilter(double alpha = 0.1);  // Constructor
    Eigen::Vector3d filter(const Eigen::Vector3d &new_value);  // Method to apply the filter

private:
    double alpha_;  // Smoothing factor
    bool initialized_;  // Flag to check if filter has been initialized
    Eigen::Vector3d filtered_value_;  // Stores the filtered value
};


class KinematicsOdometry : public rclcpp::Node
{
public:
  KinematicsOdometry();

private:
  // Subscriber to feet contact states
  rclcpp::Subscription<gazebo_msgs::msg::ContactsState>::SharedPtr feet_contact_sub_;
  void feetContactCallback(const gazebo_msgs::msg::ContactsState::SharedPtr msg);
  rclcpp::Subscription<hb40_commons::msg::RobotState>::SharedPtr feet_contact_state_sub_;
  void feetContactStateCallback(const hb40_commons::msg::RobotState::SharedPtr msg);

  // Subscriber for joint states
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Subscriber for IMU
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Velocity publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  void publishVelocity();
  void _computeLegVelocity(std::string foot_in_contact_name);
  void _computeBodyVelocity();
  rclcpp::TimerBase::SharedPtr timer_;

  // Helper for skew symmetric matrix
  Eigen::Matrix3d skewSymmetricMatrix(const Eigen::Vector3d &v);

  // Store a map of legs and the velocity they make the body move
  std::map<std::string, Eigen::Vector3d> leg_velocities_buffer_;
  Eigen::Vector3d body_linear_velocity;
  Eigen::Vector3d body_angular_velocity;

  // Maps for least squares
  // Store map for feet positions and foot velocities wrt to base frame
  std::map<std::string, Eigen::Vector3d> feet_positions_;
  std::map<std::string, Eigen::Vector3d> feet_velocities_;

  // Array to store joint states
  std::vector<double> joint_states_;
  std::vector<std::string> joint_states_names_;
  std::vector<double> joint_velocities_;
  std::map<std::string, int> pinocchio_joint_map_;

  // Array to store the names of feet for different robots
  std::map<std::string, std::string> foot_names_;

  // Array to store imu data
  std::vector<double> imu_ang_vel_;
  std::vector<double> imu_orientation_;

  // Pinocchio model
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<std::string> pinocchio_joint_names_;

  // EMA filter instance for smoothing velocity
  std::shared_ptr<EMAFilter> velocity_filter_;

  std::vector<std::string> _getKinematicChain(const std::string &link_name);
};