cmake_minimum_required(VERSION 3.5)
project(hb40_commons)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

include_directories("./")
include_directories("./include/")

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/JointCommand.msg"
  "msg/BridgeData.msg"
  "msg/BridgeState.msg"
  "msg/LegState.msg"
  "msg/RobotState.msg"
  "msg/VectorStringArray.msg"
  "msg/Status.msg"
  "srv/EnableFeature.srv"
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  geometry_msgs)

add_library(mab_commons STATIC "./src/commons_config.cpp"
                               "./src/commons_ros.cpp")
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              "rosidl_typesupport_cpp")
target_link_libraries(mab_commons nlohmann_json::nlohmann_json
                      "${cpp_typesupport_target}")
ament_target_dependencies(mab_commons rclcpp geometry_msgs)
ament_export_targets(mab_commonsTargets HAS_LIBRARY_TARGET)
ament_export_libraries(mab_commons)
ament_export_include_directories(include)

add_executable(hb40_tinyOverlord src/tinyOverlord.cpp)
target_link_libraries(hb40_tinyOverlord mab_commons)

install(DIRECTORY include/ DESTINATION include)

install(
  TARGETS mab_commons
  EXPORT mab_commonsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(TARGETS hb40_tinyOverlord DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models/urdf DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY models/rbdl DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY models/meshes DESTINATION share/${PROJECT_NAME}/models)
file(GLOB mujocoxml "models/mujoco/*.xml")
install(FILES ${mujocoxml} DESTINATION share/${PROJECT_NAME}/models/mujoco/)

ament_package()
