cmake_minimum_required(VERSION 3.8)
project(legged_kinematics_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(kinematics_odometry src/kinematics_odometry.cpp)
ament_target_dependencies(kinematics_odometry rclcpp std_msgs sensor_msgs gazebo_msgs)
target_link_libraries(kinematics_odometry pinocchio::pinocchio)

add_executable(odom_publisher src/odom_publisher.cpp)
ament_target_dependencies(odom_publisher rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

add_executable(imu_gt_fd src/imu_gt_fd.cpp)
ament_target_dependencies(imu_gt_fd rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${gazebo_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

target_include_directories(kinematics_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# target_compile_features(kinematics_odometry PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS kinematics_odometry
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS odom_publisher
  DESTINATION lib/${PROJECT_NAME})
  
  install(TARGETS imu_gt_fd
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
