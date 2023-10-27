#!/bin/bash -i

set -e

# Store the current working directory
current_dir="$(pwd)"

# Check if Folder Empty
if [[ -d ~/arena_ws ]]; then
  echo "Install Folder ~/arena_ws already exists."
  echo "This indicates Arena Rosnav is already installed."
  echo "If you wish to reinstall, please delete it."
  exit 1
fi

sudo echo ""

# Project Setup
echo "Preparing Project...:"
mkdir -p ~/arena_ws/src 
cd ~/arena_ws
catkin_make
cd src
git clone https://github.com/Arena-Rosnav/arena-rosnav.git
cd arena-rosnav
rosws update
poetry run poetry install

# Missing Deps
echo "Installing Missing Deps...:"
sudo apt update && sudo apt install -y libopencv-dev liblua5.2-dev ros-noetic-navigation ros-noetic-teb-local-planner ros-noetic-mpc-local-planner libarmadillo-dev ros-noetic-nlopt ros-noetic-turtlebot3-description ros-noetic-turtlebot3-navigation ros-noetic-lms1xx ros-noetic-velodyne-description ros-noetic-hector-gazebo ros-noetic-ira-laser-tools liblcm-dev

# Project Install
echo "Installing Project...:"
cd ../.. && catkin_make
if ! grep -q 'source $HOME/arena_ws/devel/setup.bash' ~/.bashrc; then
  echo 'source $HOME/arena_ws/devel/setup.bash' >> ~/.bashrc
fi
source ~/.bashrc

export ROS_MASTER_URI=http://127.0.0.1:11311/
export ROS_IP=127.0.0.1

pip install torch rospkg PyYAML filelock scipy PyQT5 empy defusedxml wandb lxml seaborn netifaces

# Return to the original working directory
cd "$current_dir"

echo ""
echo "Installation Complete."
echo "You can confirm that it works, by running the following command in a NEW terminal:"
echo ""
echo "roslaunch arena_bringup start_arena.launch pedsim:=true simulator:=gazebo"
