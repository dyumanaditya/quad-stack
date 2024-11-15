cmake_minimum_required(VERSION 3.8)
project(mab_control)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(hb40_commons REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${gazebo_ros_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${hb40_commons_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME} src/mab_gazebo_control_plugin.cpp)
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# install(TARGETS ${PROJECT_NAME}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

add_library(${PROJECT_NAME} SHARED src/mab_gazebo_control_plugin.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs gazebo_ros hb40_commons sensor_msgs)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} nlohmann_json::nlohmann_json)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

# add_executable(mab_gazebo_control_plugin src/mab_gazebo_control_plugin.cpp)
# target_include_directories(mab_gazebo_control_plugin PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(mab_gazebo_control_plugin PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# install(TARGETS mab_gazebo_control_plugin
#   DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
