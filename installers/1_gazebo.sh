#!/bin/bash -i
set -e

export GAZEBO_VERSION=${GAZEBO_VERSION:-harmonic}

# == install gazebo on top of ros2 ==
cd "${ARENA_WS_DIR}"

sudo apt-get install -y \
  python3-pip \
  lsb-release \
  gnupg \
  curl \
  libgps-dev

mkdir -p src/gazebo
vcs import src/gazebo < "src/arena/arena-rosnav/.repos/gazebo.repos"

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update

pushd src/gazebo
  sudo apt-get install -y \
    $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ') \
    || true
popd

rosdep install -r --from-paths src/gazebo -i -y --rosdistro ${ARENA_ROS_VERSION} \
  || echo 'rosdep failed to install all dependencies'

#install binaries for simulator and ros_gz_sim
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic

sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt install ros-jazzy-ros-gz

export GZ_VERSION=${GAZEBO_VERSION}