#!/bin/bash -i

TARGET_DIR=${1:-~/arena_ws}
branch=${2:-master}

set -e
 
# Store the current working directory
current_dir="$(pwd)"

sudo echo ""
 
# Project Setup
echo "Preparing Project...:"
mkdir -p ${TARGET_DIR}
cd ${TARGET_DIR}

# clone arena-rosnav
if [[ -d ${TARGET_DIR}/src/arena/arena-rosnav ]]; then
  cd ${TARGET_DIR}/src/arena/arena-rosnav
  git pull https://github.com/Arena-Rosnav/arena-rosnav.git ${branch}
  cd ${TARGET_DIR}
else
  git clone --branch ${branch} https://github.com/Arena-Rosnav/arena-rosnav.git src/arena/arena-rosnav
fi

until vcs import src < src/arena/arena-rosnav/.repos ; do echo "failed to update, retrying..." ; done
#

#compat
ln -s src/arena/arena-rosnav/setup/install2/* src/arena/arena-rosnav/
 
#python env init
cd src/arena/arena-rosnav
export PYTHON_KEYRING_BACKEND=keyring.backends.fail.Keyring # resolve faster
poetry run poetry install --no-root
poetry env use python3.8
. "$(poetry env info -p)/bin/activate"
cd ${TARGET_DIR}
#
 
# Missing Deps
echo "Installing Missing Deps...:"

rosdep update && rosdep install --from-paths src --ignore-src -r -y
 
# Project Install
echo "Installing Project...:"

exit 1
#TODO

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
      echo 'source ${TARGET_DIR}/devel/setup.bash' >> "$SHELL"
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