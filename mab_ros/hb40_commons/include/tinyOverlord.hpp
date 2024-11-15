#include "commons_config.hpp"

#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>


struct Ros2Process
{
    std::string package;
    std::string exec;
    bool isLaunch;
    bool isRT = false;
    int priority = 0;
    int pid = 0;
};

class TinyOverlord
{
public:
    TinyOverlord(const std::string startupConditions, const std::string& targetUser, const std::string& tgtUser);
    void perform();
    static bool sendCommandToOverlord(const std::string&command)
    {
        std::string pipePath = getTinyOverlordPipePath();
        int fd = open(pipePath.c_str(), O_WRONLY);
        write(fd, command.c_str(), command.size());
        close(fd);
        char buffer[10] = {0};
        fd = open(pipePath.c_str(), O_RDONLY);
        read(fd, buffer, 10);
        close(fd);
        if(std::string(buffer) == "ok")
            return true;
        return false;
    }
private:
    std::string targetUser = "root";
    Ros2Process procControlNode = {std::string("hb40_commons"), std::string("control.launch.py"), true, true, 48};
    Ros2Process procBridgeLaunch = {std::string("hb40_commons"), std::string("bridge.launch.py"), true, true, 49};
    Ros2Process procCameraLaunch = {std::string("ros2_mab_camera"), std::string("camera.launch.py"), true};
    Ros2Process procBag = {std::string(""), std::string("./runBag.sh"), false};

    std::string systemPipePath = "";
    char buffer[128];
    
    Ros2Process* getProcFromString(const std::string& name);

    bool startProcess(const std::string& processID, bool isRosProcess = true);
    bool stopProcess(const std::string& processID);

    static std::string getTinyOverlordPipePath()
    {
        auto config = commons::getGlobalConfig();
        if(config.contains("system_pipe_path") && config["system_pipe_path"].is_string())
            return std::string(config["system_pipe_path"]);
        throw std::runtime_error("`system_pipe_path` not found in global config. Old software/config used?");
    }
};
