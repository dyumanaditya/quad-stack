#pragma once
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <unistd.h>

#define PORT 9870

namespace commons
{
	class Plotter
	{
	  private:
		static inline int sockfd = -1;
		static inline struct sockaddr_in servaddr;
		static void init()
		{
			if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
				perror("socket creation failed");
			memset(&servaddr, 0, sizeof(servaddr));
			// Filling server information
			servaddr.sin_family = AF_INET;
			servaddr.sin_port = htons(PORT);
			servaddr.sin_addr.s_addr = INADDR_ANY;
		}

	  public:
		static void plot(std::string varName, float varValue)
		{
			if (sockfd == -1)
				init();
			nlohmann::json j;
			j[varName] = varValue;
			sendto(sockfd, (const char *)j.dump().c_str(), j.dump().length(), MSG_CONFIRM,
				(const struct sockaddr *)&servaddr, sizeof(servaddr));
		}
		void plot(std::string varName, const Eigen::Vector3f &value)
		{
			if (sockfd == -1)
				init();
			nlohmann::json j;
			j[varName]["x"] = value.x();
			j[varName]["y"] = value.y();
			j[varName]["z"] = value.z();
			sendto(sockfd, (const char *)j.dump().c_str(), j.dump().length(), MSG_CONFIRM,
				(const struct sockaddr *)&servaddr, sizeof(servaddr));
		}
	};
}
