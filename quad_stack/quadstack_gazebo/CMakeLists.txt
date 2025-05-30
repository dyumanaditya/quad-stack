
################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(quadstack_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(realsense_gazebo_plugin REQUIRED)

################################################################################
# Build
################################################################################


################################################################################
# Install
################################################################################


install(DIRECTORY launch  worlds config
  DESTINATION share/${PROJECT_NAME}/
)


################################################################################
# Macro for ament package
################################################################################

ament_package()
