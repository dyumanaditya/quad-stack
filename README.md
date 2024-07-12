# MAB ROS
ROS2 Support for MAB Robotics Silver Badger and Honey Badger robots.

# Gazebo
## Launch
Run the following line to teleoperate. 

```
ros2 launch mab_keyboard_teleop silver_badger_teleop.launch.py
```

## Run Separately
You can run all the processes in separate terminals to debug if necessary


To launch the robot in gazebo

```
ros2 launch mab_gazebo spawn_silver_badger.launch.py
```

To make the robot stand

```
ros2 run mab_stand mab_stand
```

To publish the robot states that are needed by the locomotion policy
```
ros2 run mab_utils publish_robot_state
```

To run the locomotion policy
```
ros2 run mab_locomotion mab_locomotion 
```

To run the keyboard teleop that will make the robot walk
```
ros2 run mab_keyboard_teleop teleop_node
```