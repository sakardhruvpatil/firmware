#!/bin/bash

# Ackermann Robot Deployment Script for Jetson Orin Nano
# This script sets up the ROS 2 environment on Jetson

echo "🚀 Setting up Ackermann Robot on Jetson Orin Nano..."

# Update system
sudo apt update

# Install ROS 2 Humble if not installed
if ! command -v ros2 &> /dev/null; then
    echo "📦 Installing ROS 2 Humble..."
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
    sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    sudo apt update
    sudo apt install ros-humble-desktop -y
    sudo apt install python3-argcomplete python3-colcon-common-extensions -y
fi

# Install required Python packages
echo "🐍 Installing Python dependencies..."
sudo apt install python3-matplotlib python3-numpy -y
pip3 install --user numpy matplotlib

# Install teleop twist keyboard
echo "🎮 Installing teleop keyboard..."
sudo apt install ros-humble-teleop-twist-keyboard -y

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo "✅ Jetson setup complete!"
echo ""
echo "📋 Next steps:"
echo "1. Transfer your workspace to this Jetson"
echo "2. Build the workspace: colcon build --symlink-install"
echo "3. Source: source install/setup.bash"
echo "4. Launch: ros2 launch ackermann_robot ackermann_robot_with_viz.launch.py"