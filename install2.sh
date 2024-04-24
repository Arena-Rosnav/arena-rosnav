#!/bin/bash -i

branch=${1:-master}

set -e
 
# Store the current working directory
current_dir="$(pwd)"
 
# Check if Folder Empty
if [[ -d ~/arena_ws/src/arena ]]; then
  echo "Install Folder ~/arena_ws/src/arena/arena-rosnav already exists."
  echo "This indicates Arena Rosnav is already installed."
  echo "If you wish to reinstall, please delete ~/arena_ws"
  exit 1
fi
 
sudo echo ""
 
# Project Setup
echo "Preparing Project...:"
mkdir -p ~/arena_ws
cd ~/arena_ws

# clone arena-rosnav
git clone --branch ${branch} https://github.com/Arena-Rosnav/arena-rosnav.git src/arena/arena-rosnav
until vcs import src < src/arena/arena-rosnav/.repos ; do echo "failed to update, retrying..." ; done
#
 
#python env init
cd src/arena/arena-rosnav
export PYTHON_KEYRING_BACKEND=keyring.backends.fail.Keyring # resolve faster
poetry run poetry install --no-root
poetry env use python3.8
. "$(poetry env info -p)/bin/activate"
cd ~/arena_ws
#
 
# Missing Deps
echo "Installing Missing Deps...:"

sudo apt update && sudo apt install -y libopencv-dev liblua5.2-dev libarmadillo-dev ros-noetic-nlopt liblcm-dev
rosdep update && rosdep install --from-paths src --ignore-src -r -y
 
# Project Install
echo "Installing Project...:"
catkin build

export ROS_MASTER_URI=http://127.0.0.1:11311/
export ROS_IP=127.0.0.1
 

MARKER="# ARENA-ROSNAV"
SHELLS=(~/.zshrc ~/.bashrc)

# add to .<shell>rc if exists
for SHELL in "${SHELLS[@]}"
do
  if [ -e "$SHELL" ]; then
    if ! grep -q "$MARKER" "$SHELL"; then
      echo "Adding to $SHELL"
      echo '' >> "$SHELL"
      echo "$MARKER" >> "$SHELL"
      echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$SHELL"
#      echo '. "$(cd src/arena/arena-rosnav && poetry env info -p)/bin/activate"' >> "$SHELL"
      echo 'source $HOME/arena_ws/devel/setup.bash' >> "$SHELL"
      echo '' >> "$SHELL"
    fi
  fi
done
 
# Return to the original working directory
cd "$current_dir"
 
echo ""
echo "Installation Complete."
echo "You can confirm that it works, by running the following command in a NEW terminal:"
echo ""
echo "roslaunch arena_bringup start_arena.launch"
echo ""
echo "If you need to train or use rosnav/aio planners, download and run install3_training.sh."