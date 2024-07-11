#include "commons_ros.hpp"
#include "commons_config.hpp"

using namespace commons::ros;

HeartbeatSubnode::HeartbeatSubnode(rclcpp::Node::SharedPtr node, int periodMilliseconds)
{
	nodeHandle = node;
	std::string robotPrefix = commons::getRobotNamePrefix();
	msgHeartbeat.header.frame_id = robotPrefix + "base";
	msgHeartbeat.reporting_node = nodeHandle->get_name();
	msgHeartbeat.status = msgHeartbeat.OK;

	pubHeartbeat = nodeHandle->create_publisher<hb40_commons::msg::Status>(
		robotPrefix + "heartbeat", 1);
	timHeartbeat =
		nodeHandle->create_wall_timer(std::chrono::milliseconds(periodMilliseconds),
									  std::bind(&HeartbeatSubnode::callbackTimerHeartbeat, this));
}
void HeartbeatSubnode::start() { timHeartbeat->reset(); }
void HeartbeatSubnode::stop() { timHeartbeat->cancel(); }
void HeartbeatSubnode::callbackTimerHeartbeat()
{
	msgHeartbeat.header.stamp = nodeHandle->get_clock()->now();
	pubHeartbeat->publish(msgHeartbeat);
}

StatusSubnode::StatusSubnode(rclcpp::Node::SharedPtr node)
{
	nodeHandle = node;
	std::string robotPrefix = commons::getRobotNamePrefix();
	msgStatus.header.frame_id = robotPrefix + "base";
	msgStatus.reporting_node = nodeHandle->get_name();
	msgStatus.status = msgStatus.OK;

	pubStatus = nodeHandle->create_publisher<hb40_commons::msg::Status>(
		robotPrefix + "status", 1);
}
void StatusSubnode::publishStatus(const std::string& message, unsigned char level)
{
	msgStatus.level = level;
	msgStatus.header.stamp = nodeHandle->get_clock()->now();
	msgStatus.status = message;
	pubStatus->publish(msgStatus);
}
