<p align="center">
  <img src="media/quadstack-logo-v1.png" alt="QuadStack Logo" width="200" />
</p>

<h1 align="center">QuadStack</h1>

<p align="center">
  Software stack for MAB and Unitree quadrupeds, powering robust localization, mapping and navigation:
</p>

<p align="center">
  <a href="#"><img src="https://img.shields.io/badge/⚙️-Neural%20Locomotion-blue?style=flat-square&labelColor=lightgrey" alt="Neural Locomotion" /></a>
  <a href="#"><img src="https://img.shields.io/badge/🎥-VIO%20Odometry-blue?style=flat-square&labelColor=lightgrey" alt="Visual Inertial Kinematic Aided Odometry" /></a>
  <a href="#"><img src="https://img.shields.io/badge/🗺️-SLAM-blue?style=flat-square&labelColor=lightgrey" alt="SLAM" /></a>
  <a href="#"><img src="https://img.shields.io/badge/🤖-Navigation%2C%20Planning%20%26%20Exploration-blue?style=flat-square&labelColor=lightgrey" alt="Navigation, Planning & Exploration" /></a>
</p>

---

## Features

- **Neural locomotion**  
- **Visual Inertial Kinematic aided odometry**  
- **Velocity Constrained SLAM**  
- **Navigation, Planning & Exploration**  


## Install & Setup
To clone

```bash
git clone https://github.com/dyumanaditya/quad-stack --recursive
```

To install all the necessary dependencies run the following

```bash
chmod +x setup.sh
./setup.sh
```



## Robots

This repository supports the 
- MAB Silver Badger (`silver_badger`)
- MAB Honey Badger (`honey_badger`)
- Unitree A1 (`a1`)
- Unitree Go1 (`go1`)
- Unitree Go2 (`go2`)

<div style="display: flex; justify-content: space-between;">
    <img src="./media/1.png" alt="Image 1" width="150" style="margin-right: 10px;">
    <img src="./media/2.png" alt="Image 2" width="150" style="margin-right: 10px;">
    <img src="./media/3.png" alt="Image 3" width="150" style="margin-right: 10px;">
    <img src="./media/4.png" alt="Image 4" width="150" style="margin-right: 10px;">
    <img src="./media/5.png" alt="Image 5" width="150">
</div>


To bringup a particular robot add the argument

```bash
robot:=<name>
```

To specify the initial position of the robot in the world, use the `x_pose` and `y_pose` arguments when launching.

## Worlds
By default there are two main worlds:

- AWS Small Warehouse
- AWS Small House

You can add more worlds by putting them inside the [`quadstack_gazebo/worlds`](quad_stack/quadstack_gazebo/worlds/) folder. To launch the simulation with the AWS worlds, follow the next steps

### Export the model Path
**AWS Warehouse**
```bash
export GAZEBO_MODEL_PATH=/home/ws/src/quad-stack/quad_stack/quadstack_gazebo/worlds/aws-robomaker-small-warehouse-world/models
```

**AWS House**
```bash
export GAZEBO_MODEL_PATH=/home/ws/src/quad-stack/quad_stack/quadstack_gazebo/worlds/aws-robomaker-small-house-world/models
```

### Add the world argument on bringup
**AWS Warehouse**
```bash
world:=aws-robomaker-small-warehouse-world/worlds/no_roof_small_warehouse.world
```

**AWS House**
```bash
world:=aws-robomaker-small-house-world/worlds/small_house.world
```

Custom worlds can be used similarly.


## Neural Teleoperation
To teleoperate the robot to collect rosbags or for any other purpose, use the following command. You can specify the `robot` and `world` parameter as described in the previous section

```bash
ros2 launch quadstack_bringup teleop.launch.py
```


## Visual Odometry
To launch visual odometry, the main launch line is as follows

```bash
ros2 launch quadstack_bringup odometry.launch.py
```

Along with this, there are several command line arguments that can be specified:

1. `robot` as usual
2. `world` as usual
3. `x_pose`, `y_pose` as usual for the initial robot position
4. `use_kinematics_odom` (bool, default=`true`): this allows you to use leg odometry to reset the VO if it gets lost.
5. `rosbag` (bool, default=`false`): this specifies whether a rosbag is going to be played after launching instead of real time simulation
6. `real_robot` (bool, default=`false`, only works when rosbag=`true` and `robot:=silver_badger/go2`): this specifies if the rosbag is from a real robot or from simulation


## SLAM
To launch SLAM, the main launch line is as follows

```bash
ros2 launch quadstack_bringup slam.launch.py
```

Along with this and similar to the VO launch, there are several command line arguments that can be specified:

1. `robot` as usual
2. `world` as usual
3. `x_pose`, `y_pose` as usual for the initial robot position
4. `use_kinematics_odom` (bool, default=`true`): this allows you to use leg odometry to reset the VO if it gets lost.
5. `rosbag` (bool, default=`false`): this specifies whether a rosbag is going to be played after launching instead of real time simulation
6. `real_robot` (bool, default=`false`, only works when rosbag=`true` and `robot:=silver_badger/go2`): this specifies if the rosbag is from a real robot or from simulation
7. `use_laser_stabilization` (bool, default=`true`): whether to stabilize laser scan from depth images based on IMU values
8. `use_vel_map_constraints` (bool, default=`true`): whether to add leg odometry computed velocity constraints to the SLAM factor graph.



## Navigation
To launch navigation, the main launch line is as follows

```bash
ros2 launch quadstack_bringup navigation.launch.py
```

There are two ways to perform navigation:

1. Navigation and SLAM
2. Navigation on a known map

If option 1. is used, then the launch arguments are the same as in the SLAM launch section. If not, there is an additional argument:

`map` (string, default=`''`): Allows you to specify the path to a known `yaml` map. To use this option specify the full absolute path to the map.



## Autonomous Exploration
Autononous exploration is the combined process of navigation and mapping an unknown environment. We use a frontier based exploration strategy implemented [here](https://github.com/robo-friends/m-explore-ros2). This repository should have already been installed as part of the setup process.


Launch the navigation, and in another terminal launch the following
```bash
ros2 launch explore_lite explore.launch.py
```

A separate set of SLAM and navigation parameters for exploration are available:

1.For Navigation params: [this line](https://github.com/dyumanaditya/quad-stack/blob/main/quad_stack/quadstack_navigation/launch/navigation.launch.py#L24) needs to be changed to use the file `nav2_unitree_explore.yaml` (similarly for the mab robots).
2. For SLAM params: [this line](https://github.com/dyumanaditya/quad-stack/blob/main/quad_stack/quadstack_localization/launch/slam.launch.py#L19) needs to be changed to use the file `slam_toolbox_params_explore_sim.yaml`.


## Bibtex
If you use this in your work please cite

```bibtex
@misc{aditya2025robustlocalizationmappingnavigation,
      title={Robust Localization, Mapping, and Navigation for Quadruped Robots}, 
      author={Dyuman Aditya and Junning Huang and Nico Bohlinger and Piotr Kicki and Krzysztof Walas and Jan Peters and Matteo Luperto and Davide Tateo},
      year={2025},
      eprint={2505.02272},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2505.02272}, 
}
```
