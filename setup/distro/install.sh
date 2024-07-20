#!/bin/bash -i
set -e

# Store the current working directory
current_dir="$(pwd)"

UBUNTU_VERSION='24.04'

# check if the system is running a supported ubuntu version
if [[ $(lsb_release -rs) != "${UBUNTU_VERSION}" ]]; then
  echo "Arena-Rosnav [${ARENA_BRANCH}] is intended to be run on Ubuntu "${UBUNTU_VERSION}", but you are running:"
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

sudo add-apt-repository universe
sudo apt-get update
sudo apt-get install -y curl

echo "Installing tzdata...:"
export DEBIAN_FRONTEND=noninteractive
sudo apt install -y tzdata && sudo dpkg-reconfigure --frontend noninteractive tzdata

# ROS
echo "Setting up ROS2 ${ARENA_ROS_VERSION}...:"

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


# Getting Packages
echo "Installing Deps...:"

sudo apt install -y $(cat package.list)

source "/opt/ros/${ARENA_ROS_VERSION}/setup.bash"

# pyenv
if [ ! -d ~/.pyenv ]; then
  curl https://pyenv.run | bash
  echo 'export PYENV_ROOT="$HOME/.pyenv"'                                 >> ~/.bashrc
  echo '[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"'  >> ~/.bashrc
  echo 'eval "$(pyenv init -)"'                                           >> ~/.bashrc
  source ~/.bashrc
fi

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
else
  sudo rosdep init
fi

rosdep update

# # Return to the original working directory
# cd "$current_dir"
# 
# echo ""
# echo "Now please run the second install script in a NEW terminal."
# echo "You NEED to open the new terminal AFTER this script finishes."
# echo "You can run the second script with the following command:"
# echo ""
# echo "curl https://raw.githubusercontent.com/Arena-Rosnav/arena-rosnav/master/install2.sh | bash"