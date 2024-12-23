# quad-stack
Software stack MAB and Unitree Quadrupeds

# Usage

Here are the commands to run the package. The following launch arguments are supported

- `robot` (silver_badger, honey_badger, a1, go1, go2)
- `world` (any world file in the gazebo worlds folder)
- `x_pose`, `y_pose`, `z_pose`


### Spawning the robot

```bash
ros2 launch quadstack_bringup spawn_robot.launch.py world:=empty_world robot:=a1
```

### Teleoperating the robot

```bash
ros2 launch quadstack_bringup teleop.launch.py world:=empty_world robot:=a1
```

### Visual and Legged Odometry

```bash
ros2 launch quadstack_bringup odometry.launch.py robot:=silver_badger
```
