#pragma once

#include "commons_math.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nlohmann/json.hpp"
#include "hb40_commons/msg/status.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

#include <fstream>
#include <iostream>
#include <libgen.h>
#include <string>
#include <unistd.h>

namespace commons
{
	namespace ros
	{
		inline geometry_msgs::msg::Vector3 rosVecFromArray(const float* array)
		{
			geometry_msgs::msg::Vector3 rosVec;
			rosVec.x = array[0];
			rosVec.y = array[1];
			rosVec.z = array[2];
			return rosVec;
		}
		inline geometry_msgs::msg::Vector3 rosVecFromEigenVec(const v3& eigenVec)
		{
			geometry_msgs::msg::Vector3 rosVec;
			rosVec.x = eigenVec.x();
			rosVec.y = eigenVec.y();
			rosVec.z = eigenVec.z();
			return rosVec;
		}
		inline geometry_msgs::msg::Vector3 rosVecFromRosPoint(const geometry_msgs::msg::Point& rosPoint)
		{
			geometry_msgs::msg::Vector3 rosVec;
			rosVec.x = rosPoint.x;
			rosVec.y = rosPoint.y;
			rosVec.z = rosPoint.z;
			return rosVec;
		}
		inline geometry_msgs::msg::Point rosPointFromEigenVec(const v3& eigenVec)
		{
			geometry_msgs::msg::Point rosVec;
			rosVec.x = eigenVec.x();
			rosVec.y = eigenVec.y();
			rosVec.z = eigenVec.z();
			return rosVec;
		}
		inline geometry_msgs::msg::Quaternion rosQuatFromEigenQuat(const quat& eigenQuat)
		{
			geometry_msgs::msg::Quaternion rosQuat;
			rosQuat.x = eigenQuat.x();
			rosQuat.y = eigenQuat.y();
			rosQuat.z = eigenQuat.z();
			rosQuat.w = eigenQuat.w();
			return rosQuat;
		}
		inline v3 eigenVecFromRosVec(const geometry_msgs::msg::Vector3& rosVec)
		{
			return v3(rosVec.x, rosVec.y, rosVec.z);
		}
		inline v3 eigenVecFromRosPoint(const geometry_msgs::msg::Point& rosVec)
		{
			return v3(rosVec.x, rosVec.y, rosVec.z);
		}
		inline quat eigenQuatFromRosQuat(const geometry_msgs::msg::Quaternion& rosQuat)
		{
			return quat(rosQuat.w, rosQuat.x, rosQuat.y, rosQuat.z);
		}
		inline vX eigenVecFromStdVec(const std::vector<double>& stdVec)
		{
			vX result;
			result.resize(stdVec.size());
			for (u32 i = 0; i < stdVec.size(); i++)
				result[i] = stdVec[i];
			return result;
		}
		inline vX eigenVecFromStdVec(const std::vector<float>& stdVec)
		{
			vX result;
			result.resize(stdVec.size());
			for (u32 i = 0; i < stdVec.size(); i++)
				result[i] = stdVec[i];
			return result;
		}
		inline rclcpp::QoS getMabQosRT()
		{
			rclcpp::QoS qos(1);
			qos.keep_last(1);
			qos.best_effort();
			qos.durability_volatile();
			return qos;
		}
		inline rclcpp::QoS getMabQosReliable()
		{
			rclcpp::QoS qos(1);
			qos.keep_last(1);
			qos.reliable();
			qos.durability_volatile();
			return qos;
		}
		class HeartbeatSubnode
		{
		  private:
			rclcpp::Node::SharedPtr nodeHandle;
			rclcpp::Publisher<hb40_commons::msg::Status>::SharedPtr pubHeartbeat;
			rclcpp::TimerBase::SharedPtr timHeartbeat;
			hb40_commons::msg::Status msgHeartbeat;
			void callbackTimerHeartbeat();

		  public:
			HeartbeatSubnode(rclcpp::Node::SharedPtr nodeHandle, int periodMilliseconds);
			void start();
			void stop();
		};
		class StatusSubnode
		{
		  private:
			rclcpp::Node::SharedPtr nodeHandle;
			rclcpp::Publisher<hb40_commons::msg::Status>::SharedPtr pubStatus;
			hb40_commons::msg::Status msgStatus;

		  public:
			static constexpr unsigned char OK = hb40_commons::msg::Status::OK;
			static constexpr unsigned char STALE = hb40_commons::msg::Status::STALE;
			static constexpr unsigned char WARN = hb40_commons::msg::Status::WARN;
			static constexpr unsigned char ERROR = hb40_commons::msg::Status::ERROR;

			StatusSubnode(rclcpp::Node::SharedPtr nodeHandle);
			void publishStatus(const std::string& message, unsigned char level);
		};
	} // namespace ros
} // namespace commons
