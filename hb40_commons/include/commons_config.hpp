
#pragma once
#include "commons_math.hpp"
#include <nlohmann/json.hpp>

namespace commons
{
    bool readGlobalConfig();
    const nlohmann::json& getGlobalConfig();
    bool updateGlobalConfig(const nlohmann::json& newConfig);
    template <typename T>
    T getParameterFromConfig(const char* parameterName)
    {
        auto& config = getGlobalConfig();
        if(!config.contains(parameterName))
            throw std::invalid_argument(std::string("Config does not contain parameter: ") + parameterName);
        if(std::is_arithmetic<T>() && config[parameterName].is_number())
            return config[parameterName].get<T>();
        if(std::is_same<T, std::string>() && config[parameterName].is_string())
            return config[parameterName].get<T>();
        throw std::invalid_argument(std::string("Parameter is neither a number nor a string!"));
    }
    template <typename T>
    T getParameterFromConfig(const char* parameterName, int index)
    {
        auto& config = getGlobalConfig();
        if(!config.contains(parameterName) || !config[parameterName].is_array())
            throw std::invalid_argument(std::string("Config does not contain array parameter: ") + parameterName);
        return config[parameterName][index];
    }
    std::vector<float> getParameterVectorFromConfig(const char* parameterName);
	v3 getParameterV3FromConfig(const char* parameterName);

    std::string getRobotNamePrefix(bool omitSlash = false);
    std::string getPathToGlobalStorage();
    bool checkIfFileExists(const std::string& filepath);
}
