#!/bin/bash -i

set -e

# Store the current working directory
current_dir="$(pwd)"

# Check if the system is not running Ubuntu 20.04
if [[ $(lsb_release -rs) != "20.04" ]]; then
  echo "Arena Rosnav is intended to be run on Ubuntu 20.04, but you are running:"
  echo $(lsb_release -a)
  echo "This may result in the installation failing."

  # Ask the user if they want to continue
  read -p "Do you want to continue anyway? (Y/n): " choice
  choice="${choice:-Y}"
  if [[ "$choice" =~ ^[Yy] ]]; then
    echo "Continuing..."
  else
    echo "Exiting script."
    exit 1
  fi
fi
# Check if Folder Empty
if [[ -d ~/arena_ws ]]; then
  echo "Install Folder ~/arena_ws already exists."
  echo "This indicates Arena Rosnav is already installed."
  echo "If you wish to reinstall, please delete it."
  exit 1
fi

sudo apt update
sudo apt install -y curl

# ROS
echo "Installing ROS...:"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Getting Packages
echo "Installing Deps...:"
sudo apt install -y python3 python-is-python3 git python3-rosdep python3-pip python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Poetry
echo "Installing Poetry...:"
curl -sSL https://install.python-poetry.org | python3 -
export PATH="$HOME/.local/bin:$PATH"


# Check if the default ROS sources.list file already exists
ros_sources_list="/etc/ros/rosdep/sources.list.d/20-default.list"
if [[ -f "$ros_sources_list" ]]; then
  echo "rosdep appears to be already initialized"
  echo "Default ROS sources.list file already exists:"
  echo "$ros_sources_list"
  
  # Ask the user if they want to delete the existing file
  read -p "Do you want to keep the existing sources.list file or re-initialize rosdep? (K/r): " init_choice
  init_choice="${init_choice:-K}"
  if [[ "$init_choice" =~ ^[Kk] ]]; then
    echo "Keeping File."
  else
    echo "Re-Initializing...:"
    sudo rm "$ros_sources_list"
    sudo rosdep init
  fi
  else
    sudo rosdep init
fi

rosdep update

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
sudo apt update && sudo apt install -y libopencv-dev liblua5.2-dev ros-noetic-navigation ros-noetic-teb-local-planner ros-noetic-mpc-local-planner libarmadillo-dev ros-noetic-nlopt ros-noetic-turtlebot3-description ros-noetic-turtlebot3-navigation ros-noetic-lms1xx ros-noetic-velodyne-description ros-noetic-hector-gazebo ros-noetic-ira-laser-tools 

# Project Install
echo "Installing Project...:"
cd ../.. && catkin_make
echo "source $HOME/arena_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

export ROS_MASTER_URI=http://127.0.0.1:11311/
export ROS_IP=127.0.0.1

pip install torch rospkg PyYAML filelock scipy PyQT5 empy defusedxml wandb lxml seaborn netifaces

echo ""
echo "Installation Complete."
echo "You can confirm that it works, by running:"
echo "roslaunch arena_bringup start_arena.launch pedsim:=true simulator:=gazebo"

# Return to the original working directory
cd "$current_dir"
