#!/bin/bash -i
 
set -e
 
# Store the current working directory
current_dir="$(pwd)"
 
# Check if Folder Empty
if [[ -d ~/arena_ws/src/arena-rosnav ]]; then
  echo "Install Folder ~/arena_ws/src/arena-rosnav already exists."
  echo "This indicates Arena Rosnav is already installed."
  echo "If you wish to reinstall, please delete ~/arena_ws"
  exit 1
fi
 
sudo echo ""
 
# Project Setup
echo "Preparing Project...:"
mkdir -p ~/arena_ws/src 
cd ~/arena_ws/src
git clone https://github.com/Arena-Rosnav/arena-rosnav.git
cd arena-rosnav
until rosws update ; do echo "failed to update, retrying..." ; done
 
 
#python env init
poetry run poetry install
#
 
# Missing Deps
echo "Installing Missing Deps...:"
cd ../..
sudo apt update && sudo apt install -y libopencv-dev liblua5.2-dev libarmadillo-dev ros-noetic-nlopt liblcm-dev
rosdep update && rosdep install --from-paths src --ignore-src -r -y
 
# Project Install
echo "Installing Project...:"
catkin build
 
export ROS_MASTER_URI=http://127.0.0.1:11311/
export ROS_IP=127.0.0.1
 
python -m pip install torch rospkg PyYAML filelock scipy PyQT5 empy defusedxml wandb lxml seaborn netifaces

# add to .bashrc if exists
if [ -e ~/.zshrc ]; then
  if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.zshrc; then
    echo "source /opt/ros/noetic/setup.bash" >> ~/.zshrc
  fi
  if ! grep -q 'export PATH="$HOME/.local/bin"' ~/.zshrc; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc
  fi
  if ! grep -q 'source $HOME/arena_ws/devel/setup.bash' ~/.zshrc; then
    echo 'source $HOME/arena_ws/devel/setup.bash' >> ~/.zshrc
  fi
fi

# add to .zshrc if exists
if [ -e ~/.zshrc ]; then
  if ! grep -q "source /opt/ros/noetic/setup.bash" ~/.zshrc; then
    echo "source /opt/ros/noetic/setup.bash" >> ~/.zshrc
  fi
  if ! grep -q 'export PATH="$HOME/.local/bin"' ~/.zshrc; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc
  fi
  if ! grep -q 'source $HOME/arena_ws/devel/setup.bash' ~/.zshrc; then
    echo 'source $HOME/arena_ws/devel/setup.bash' >> ~/.zshrc
  fi
fi
 
# Return to the original working directory
cd "$current_dir"
 
echo ""
echo "Installation Complete."
echo "You can confirm that it works, by running the following command in a NEW terminal:"
echo ""
echo "roslaunch arena_bringup start_arena.launch"
