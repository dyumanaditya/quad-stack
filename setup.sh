#!/bin/bash

# Update the package index
sudo apt update

# Install Gazebo packages
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros

# Install nlohmann-json3-dev
sudo apt install -y nlohmann-json3-dev

# # Install TurtleBot3 packages
# sudo apt install -y ros-humble-turtlebot3*

# Install Python3 and pip
sudo apt install -y python3 python3-pip

# Install Python packages using pip
pip3 install -U jax
pip3 install flax
pip3 install opencv-python
pip3 install transforms3d
pip3 install matplotlib
pip3 install catkin-pkg
pip3 install empy==3.3.4
pip3 install lark
pip3 install lxml

# Install xterm
sudo apt install -y xterm

# Install colcon common extensions
sudo apt install -y python3-colcon-common-extensions

# Install ROS Humble image rotate package
sudo apt install -y ros-humble-image-rotate

# Install ROS Humble robot localization package
sudo apt-get install -y ros-humble-robot-localization

# Install ROS Humble RTAB-Map package
sudo apt install -y ros-humble-rtabmap*

# Install ROS Humble TF2 ROS package
sudo apt-get install -y ros-humble-tf2-ros ros-humble-tf2-tools
sudo apt install -y ros-humble-tf-transformations

sudo apt-get install -y ros-humble-rcl-interfaces ros-humble-point-cloud-transport
sudo apt install -y ros-humble-rosidl-generator-dds-idl

pip install -U "jax[cuda12]"

pip install numpy==1.26.4
sudo apt install -y ros-humble-pinocchio

# Clone AWS worlds (warehouse only works on branch ros1)
git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-warehouse-world quad_stack/quadstack_gazebo/worlds/
git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-house-world quad_stack/quadstack_gazebo/worlds/

# Clone the modified slam package
git clone https://github.com/dyumanaditya/slam_toolbox

# Clone explore package
git clone https://github.com/dyumanaditya/m-explore-ros2

echo "Installation of all packages for quad-stack is complete!"

