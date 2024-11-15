#include "commons_config.hpp"
#include "commons_math.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <type_traits>

uint64_t getFileModTime(const char* path)
{
	struct stat result;
	if (stat(path, &result) == 0)
		return (uint64_t)result.st_mtime;
	return 0;
}

namespace commons
{
	nlohmann::json global;
	bool init = false;

	std::string getPathToGlobalConfig()
	{
		std::string homePath = std::string(getenv("HOME"));
		return std::string(homePath + std::string("/.ros/mab/config/global.json"));
	}

	bool readGlobalConfig()
	{
        static long configLastModTs = 0;
        long configModTs = getFileModTime(getPathToGlobalConfig().c_str());
        if(configLastModTs == configModTs)
            return false; 
        configLastModTs = configModTs;
		std::string globalConfigPath = getPathToGlobalConfig();
		std::ifstream configx(globalConfigPath);
		if (configx.fail())
		{
			std::cout << "#############################################################################"
					  << std::endl
					  << std::endl;
			std::cout << "FAILED TO OPEN GLOBAL CONFIG FILE! BUILDING WITH `build.sh` SHOULD SOLVE IT!"
					  << std::endl
					  << std::endl;
			std::cout << "#############################################################################"
					  << std::endl;
			exit(1);
		}
        std::cout << "Reading global config!" << std::endl;
		init = true;
		configx >> global;
		return init;
	}
	const nlohmann::json& getGlobalConfig()
	{
		if (!init) 
			readGlobalConfig();
		return global;
	}
	bool updateGlobalConfig(const nlohmann::json& newConfig)
	{
		auto config = getGlobalConfig();
		config.update(newConfig);
		std::string globalConfigPath = getPathToGlobalConfig();
		std::ofstream configf(globalConfigPath);
		configf << std::setw(4) << config;
		return true;
	}
	std::vector<float> getParameterVectorFromConfig(const char* parameterName)
	{
		auto& config = getGlobalConfig();
		if (!config.contains(parameterName) || !config[parameterName].is_array())
			throw std::invalid_argument(std::string("Config does not contain array parameter: ") +
										parameterName);
		return config[parameterName];
	}
	v3 getParameterV3FromConfig(const char* parameterName)
	{
		std::vector<float> v = getParameterVectorFromConfig(parameterName);
		if (v.size() != 3)
			throw std::invalid_argument(std::string("Config does not contain v3: ") + parameterName);
		return v3(v[0], v[1], v[2]);
	}
	std::string getRobotNamePrefix(bool omitSlash)
	{
		std::string robotNamePrefix = "";
		auto config = commons::getGlobalConfig();
		if (config.contains("robot_name") && config["robot_name"].is_string())
		{
			robotNamePrefix = config["robot_name"];
            if(omitSlash)
                return robotNamePrefix;
			robotNamePrefix += "/";
		}
		return robotNamePrefix;
	}
	std::string getPathToGlobalStorage()
	{
		std::string globalStoragePath = std::string(getenv("HOME"));
		globalStoragePath += "/.ros/mab/";
		return globalStoragePath;
	}
	bool checkIfFileExists(const std::string& filePath)
	{
		struct stat buffer;
		return (stat(filePath.c_str(), &buffer) == 0);
	}

} // namespace commons
