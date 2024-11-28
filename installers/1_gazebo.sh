#!/bin/bash -i
set -e

# Set Gazebo version if not provided
export GAZEBO_VERSION=${GAZEBO_VERSION:-harmonic}

# Define Arena workspace directory
cd "${ARENA_WS_DIR}"

# Install dependencies
sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  lsb-release \
  gnupg \
  curl \
  libgps-dev

# Add OSRF Gazebo packages repository
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Add ROS 2 repository
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > \
  /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update package lists
sudo apt-get update

# Install Gazebo binaries and ROS-Gazebo bridge
sudo apt-get install -y \
  gz-${GAZEBO_VERSION} \
  ros-${ARENA_ROS_DISTRO}-ros-gz

# Set Gazebo version environment variable
export GZ_VERSION=${GAZEBO_VERSION}

echo "Gazebo ${GAZEBO_VERSION} and ROS-Gazebo bridge installed successfully!"
