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

sudo add-apt-repository universe
sudo apt update

# ROS
echo "Installing ROS...:"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full
if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
fi
source ~/.bashrc

# Getting Packages
echo "Installing Deps...:"
sudo apt install -y python3 python-is-python3 git python3-rosdep python3-pip python3-rosinstall-generator python3-vcstool build-essential python3-catkin-tools

# Poetry
echo "Installing Poetry...:"
curl -sSL https://install.python-poetry.org | python3 -
if ! grep -q 'export PATH="$HOME/.local/bin"' ~/.bashrc; then
  echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
fi


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

# Return to the original working directory
cd "$current_dir"

echo ""
echo "Now please run the second install script in a NEW terminal."
echo "You NEED to open the new terminal AFTER this script finishes."
echo "You can run the second script with the following command:"
echo ""
echo "curl https://raw.githubusercontent.com/Arena-Rosnav/arena-rosnav/master/install2.sh | bash"
