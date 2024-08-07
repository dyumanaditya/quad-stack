# MAB ROS
ROS2 Support for MAB Robotics Silver Badger and Honey Badger robots.

# Setup
To install all the necessary dependencies run the following

```bash
chmod +x setup.sh
./setup.sh
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
