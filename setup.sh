#!/bin/bash

# Update the package index
sudo apt update

# Install Gazebo packages
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros

# Install nlohmann-json3-dev
sudo apt install -y nlohmann-json3-dev

# Install TurtleBot3 packages
sudo apt install -y ros-humble-turtlebot3*

# Install Python3 and pip
sudo apt install -y python3 python3-pip

# Install Python packages using pip
pip3 install -U jax
pip3 install flax
pip3 install opencv-python
pip3 install transforms3d
pip3 install matplotlib

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

sudo apt-get install -y ros-humble-rcl-interfaces

pip install -U "jax[cuda12]"


echo "Installation of all packages for mab_ros is complete!"
