#include "tinyOverlord.hpp"

#include <chrono>
#include <iostream>
#include <thread>
#include <wait.h>
#include <grp.h>
#include <pwd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <sys/resource.h>
// #define TO_DEBUG

unsigned int getIpAddres(const char* interfaceName)
{
	struct ifreq ifr;
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	ifr.ifr_addr.sa_family = AF_INET; // Ipv4
	strncpy(ifr.ifr_name, interfaceName, IFNAMSIZ - 1);
	ioctl(fd, SIOCGIFADDR, &ifr);
	close(fd);
	unsigned int ip = ((struct sockaddr_in*)&ifr.ifr_ifru.ifru_addr)->sin_addr.s_addr;
#ifdef TO_DEBUG
	std::cout << "TO: Network address: " << std::hex << "0x" << ip << std::dec << std::endl;
	in_addr adr;
	adr.s_addr = ip;
	std::cout << "TO: Network address: " << inet_ntoa(adr) << std::endl;
#endif
	return ip;
}

bool waitForNetwork(const char* interfaceName, int timeoutSeconds)
{
	std::cout << "Waiting for network..." << std::endl;
	auto start = std::chrono::high_resolution_clock::now();
#ifdef TO_DEBUG
	using namespace std::chrono;
	std::cout << "TO: Waiting for network... " << std::endl
			  << "TO: Start: " << duration_cast<hours>(start.time_since_epoch()).count() << ":"
			  << duration_cast<minutes>(start.time_since_epoch()).count() << ":"
			  << duration_cast<seconds>(start.time_since_epoch()).count() << std::endl;
#endif
	auto stop = start + std::chrono::seconds(timeoutSeconds);
	while (std::chrono::duration_cast<std::chrono::milliseconds>(
			   stop - std::chrono::high_resolution_clock::now())
			   .count() > 0)
	{
		unsigned int adr = getIpAddres(interfaceName);
#ifdef TO_DEBUG
		std::cout << "Waiting " << sec++ << std::endl;
		std::cout << "Adr returned: " << adr << std::endl;
#endif
		if (adr > 0xffff)
		{
			in_addr ip;
			ip.s_addr = adr;
			std::cout << "Netowork connected with ip " << inet_ntoa(ip) << std::endl;
			return true;
		}
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	std::cout << "Waiting for network failed: TIME OUT" << std::endl;
	return false;
}

pid_t startProcessExec(bool isLaunchFile, std::string& package, std::string& executable,
	const char* username, bool isRosExec = true)
{
	if (strcmp(username, "root") == 0)
	{
		std::cout << "###### TinyOverlord told to execute command as 'root'! Will not do that! ######"
				  << std::endl;
		return -1;
	}

	pid_t pid = fork();
	if (pid != 0)
		return pid;

	// Get info about target user
	int nGroups = 20;
	gid_t groups[20];
	auto user = getpwnam(username);
#ifdef TO_DEBUG
	std::cout << "TO: user->name: " << user->pw_name << std::endl;
	std::cout << "TO: user->uid: " << user->pw_uid << std::endl;
	std::cout << "TO: user->gid: " << user->pw_gid << std::endl;
	std::cout << "TO: user->gecos: " << user->pw_gecos << std::endl;
	std::cout << "TO: user->dir: " << user->pw_dir << std::endl;
	std::cout << std::flush;
#endif
	if (user->pw_uid <= 0)
		return -1;
	getgrouplist(username, user->pw_gid, groups, &nGroups);

	// set process user to match target
	setgroups(nGroups, groups);
	setgid(user->pw_gid);
	setuid(user->pw_uid);
	std::string homeDir = "/home/" + std::string(username);
	putenv((char*)homeDir.c_str());

	// run command as user
	std::string command = "run";
	if (isLaunchFile)
		command = "launch";

#ifdef TO_DEBUG
	std::cout << "TO: isRosExec: " << isRosExec << std::endl;
	std::cout << "TO: Starting Process: ros2 " << command << " " << package << " " << executable
			  << std::endl;
	std::cout << "TO: HOME path set to: " << homeDir << std::endl;
	std::cout << "TO: as user: " << username << " (uid: " << user->pw_uid << ", gid: " << user->pw_gid
			  << ") member of " << nGroups << " groups: " << std::endl
			  << "TO: ";
	for (int i = 0; i < nGroups; i++)
		std::cout << (unsigned int)groups[i] << ", ";
	std::cout << std::endl << std::flush;
#endif
	int nArgs = 0;
	char** args = NULL;
	args = (char**)malloc(5 * sizeof(char*));
	if (isRosExec)
	{
		nArgs = 5;
		args[0] = (char*)malloc(sizeof(char) * 5);
		strcpy(args[0], "ros2");
		args[1] = (char*)malloc(sizeof(char) * command.size() + 1);
		strcpy(args[1], command.c_str());
		args[2] = (char*)malloc(sizeof(char) * package.size() + 1);
		strcpy(args[2], package.c_str());
		args[3] = (char*)malloc(sizeof(char) * executable.size() + 1);
		strcpy(args[3], executable.c_str());
		args[4] = (char*)malloc(sizeof(char) * 1);
		args[4] = NULL;
	}
	else
	{
		nArgs = 2;
		std::string execPath = homeDir + "/install/" + executable;
		args[0] = (char*)malloc(sizeof(char) * execPath.size() + 1);
		strcpy(args[0], execPath.c_str());
		args[1] = (char*)malloc(sizeof(char) * 1);
		args[1] = NULL;
	}
	std::cout << "TO: Starting execution of: ";
	for (int i = 0; i < nArgs; i++)
		std::cout << args[i] << " ";
	std::cout << std::endl << std::flush;

	execvp(args[0], (char* const*)args);
	std::cout << "#### ERROR: Process EXITED! ####" << std::endl;
	exit(0);
}

TinyOverlord::TinyOverlord(const std::string startupConditions, const std::string& tgtUser,
	const std::string& networkInterface)
{
	targetUser = tgtUser;
	systemPipePath = getTinyOverlordPipePath();
	std::cout << "PipePath: " << systemPipePath << std::endl;
	if (!commons::checkIfFileExists(systemPipePath))
	{
		auto oldUmask = umask(0000);
		mkfifo(systemPipePath.c_str(), (0666));
		umask(oldUmask);
	}

	// NOTE: Below network related operations are required of ROS2
	//  If node is started before network interface receive IP address
	//  e.g. starting before network router
	std::string cmd = startupConditions;
	if (startupConditions == "bridge")
	{
		if (!waitForNetwork(networkInterface.c_str(), 10))
		{
			std::cerr << "Could not connect to network in 10 seconds! Retrying..." << std::endl;
			if (waitForNetwork(networkInterface.c_str(), 50))
				std::cerr << "Could not connect to network in 60 seconds! Starting without network..."
						  << std::endl;
		}

		startProcess(cmd);
		cmd = "control";
		sleep(5);
		startProcess(cmd);
	}
	if (startupConditions == "camera")
	{
		if (!waitForNetwork(networkInterface.c_str(), 100))
			std::cout << "Could not get network after 100s! Timeout!" << std::endl;
		startProcess(cmd);
	}
}
Ros2Process* TinyOverlord::getProcFromString(const std::string& processID)
{
	if (processID == "control")
		return &procControlNode;
	if (processID == "bridge")
		return &procBridgeLaunch;
	if (processID == "camera")
		return &procCameraLaunch;
	if (processID == "bag")
		return &procBag;
	return nullptr;
}
bool TinyOverlord::startProcess(const std::string& processID, bool isRosProcess)
{
	Ros2Process* p = getProcFromString(processID);
	if (p->pid != 0)
		return false;

	p->pid = startProcessExec(p->isLaunch, p->package, p->exec, targetUser.c_str(), isRosProcess);
	if (p->pid > 0)
	{
		if (p->isRT)
		{
			struct sched_param param;
			param.sched_priority = p->priority;
			if (sched_setscheduler(p->pid, SCHED_FIFO, &param))
			{
				std::cout << "TO: Failed to set process " << (int)p->pid << " aka " << processID
						  << " to SCHED_FIFO!" << std::endl;
				std::cout << std::strerror(errno) << std::endl;
			}
		}
		return true;
	}
	std::cout << "TO: Failed to start process: " << processID << std::endl;
	return false;
}
bool TinyOverlord::stopProcess(const std::string& processID)
{
	Ros2Process* p = getProcFromString(processID);

	system(std::string("pkill --signal 2 -P " + std::to_string(p->pid)).c_str());
	int tmp;
	if (waitpid(p->pid, &tmp, 0) == p->pid)
	{
		p->pid = 0;
		return true;
	}
	return false;
}

void TinyOverlord::perform()
{
	memset(buffer, 0, sizeof(buffer));
	int fd = open(systemPipePath.c_str(), O_RDONLY);
	if (fd <= 0)
		return;
	read(fd, buffer, 100);
	close(fd);
	std::string msg(buffer);
	std::cout << "TO received: " << msg << std::endl;
	std::vector<std::string> commands;
	std::string delimiter = ";";
	size_t pos = 0;
	std::string token;
	while ((pos = msg.find(delimiter)) != std::string::npos)
	{
		token = msg.substr(0, pos);
		commands.push_back(token);
		msg.erase(0, pos + delimiter.length());
	}
	if (commands.size() != 2)
		return;

	bool success = false;
	bool shouldShutdown = false;
	bool isRosProcess = (commands[1] == "bag" ? false : true);
	if (commands[0] == "start")
		success = startProcess(commands[1], isRosProcess);
	if (commands[0] == "stop")
		success = stopProcess(commands[1]);
	if (commands[0] == "shutdown")
		shouldShutdown = success = true;

	std::string response = (success ? "ok" : "fail");
	fd = open(systemPipePath.c_str(), O_WRONLY);
	write(fd, response.c_str(), response.size());
	close(fd);

	if (shouldShutdown)
	{
		sleep(2);
		system("shutdown now");
	}
}

int main(int argc, const char** argv)
{
	std::string startup = "", targetUser = "", networkInterface = "";
	if (argc < 4)
		std::cout << "Starting TinyOverlord in TEST mode with no startup conditions!" << std::endl;
	else
	{
		startup = std::string(argv[1]);
		std::cout << "Starting TinyOverlord with " << startup << " startup condition." << std::endl;
		targetUser = std::string(argv[2]);
		std::cout << "Child processes will be run as user: " << targetUser << std::endl;
		networkInterface = std::string(argv[3]);
		std::cout << "Using NetworkInterface: " << networkInterface << std::endl;
	}

	TinyOverlord to(startup, targetUser, networkInterface);
	while (1)
		to.perform();
	return 0;
}
