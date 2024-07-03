#!/bin/bash -i

set -e

# Store the current working directory
current_dir="$(pwd)"

TARGET_DIR=${1:-~/arena_ws}

echo $TARGET_DIR

# Check if the system is not running Ubuntu 22.04
if [[ $(lsb_release -rs) != "22.04" ]]; then
  echo "Arena Rosnav is intended to be run on Ubuntu 22.04, but you are running:"
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
if [[ -d ${TARGET_DIR} ]]; then
  echo "Install Folder ${TARGET_DIR} already exists."
  echo "This indicates Arena Rosnav is already installed."
  echo "If you wish to reinstall, please delete it."
  exit 1
fi

sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl

# ROS
echo "Installing ROS2 Humble...:"

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
source ~/.bashrc

# Getting Packages
echo "Installing Deps...:"

sudo apt install -y $(curl https://raw.githubusercontent.com/Arena-Rosnav/arena-rosnav/ros2/setup/install1/package.list)

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
