# MAB ROS
ROS2 Support for MAB Robotics Silver Badger and Honey Badger robots.

# Install
To clone

```bash
git clone https://github.com/boris-il-forte/mab_ros --recursive
```

# Setup
To install all the necessary dependencies run the following

```bash
chmod +x setup.sh
./setup.sh
```

You will also need the `legged_kinematic_odometry` package in `src`. Contact Dyuman Aditya if you need access

```bash
git clone https://github.com/dyumanaditya/legged_kinematics_odometry
```

## Bug in Nav2
Due to [this bug](https://github.com/ros-navigation/navigation2/issues/3644#issuecomment-1614553365)

In `/opt/ros/humble/share/nav2_bringup/launch/localization_launch.py`  Change
```yaml
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params],
                remappings=remappings),
```

to 

```yaml
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params,
                            {'yaml_filename': map_yaml_file}], # Added this to parameters
                remappings=remappings),
```

# Gazebo

To spawn the robot

```bash
ros2 launch mab_bringup mab_spawn.launch.py
```

To spawn the robot with teleop and a neural locomotion control policy

```bash
ros2 launch mab_bringup mab_teleop.launch.py
```


To spawn the robot with teleop and a neural locomotion control policy and localization (visual odometry and SLAM)

```bash
ros2 launch mab_bringup mab_localization.launch.py
```


To spawn the robot with the localization stack and navigation stack (ros nav2)

```bash
ros2 launch mab_bringup mab_localization.launch.py
```
