std_msgs/Header header
LegState[] leg
geometry_msgs/Pose pose
geometry_msgs/Twist body_vel
geometry_msgs/Twist world_vel

# joint positions, velocities, and torques, dimensions are 20, 19, 19
float64[] joint_pos
float64[] joint_vel
float64[] joint_torque