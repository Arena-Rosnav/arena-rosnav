#!/bin/bash -i
set -e
 
PYTHON_VERSION=3.10

sudo echo ""

# build ros2 from source
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

mkdir -p "${ARENA_WS_DIR}/src/ros2"
cd "${ARENA_WS_DIR}"
wget "https://raw.githubusercontent.com/ros2/ros2/${ARENA_ROS_VERSION}/ros2.repos"
vcs import src < ros2.repos
rosdep install --from-paths src --ignore-src --rosdistro ${ARENA_ROS_VERSION} -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"

# Project Setup
echo "Preparing Project...:"
cd "${ARENA_WS_DIR}"

until vcs import src < src/arena/arena-rosnav/.repos ; do echo "failed to update, retrying..." ; done
#

#compat
ln -rfs src/arena/arena-rosnav/setup/install2/* src/arena/arena-rosnav/

#python env
pyenv install -s "${PYTHON_VERSION}"
pyenv local "${PYTHON_VERSION}"
cd src/arena/arena-rosnav/setup/arena
$HOME/.local/bin/poetry env use "${PYTHON_VERSION}"
export PYTHON_KEYRING_BACKEND=keyring.backends.fail.Keyring 
$HOME/.local/bin/poetry install || ($HOME/.local/bin/poetry lock --no-update && $HOME/.local/bin/poetry install)

# Missing Deps
sudo apt install -y $(cat package.list)

cd ${ARENA_WS_DIR}
echo "Installing Missing Deps...:"
rosdep update && rosdep install --from-paths src --ignore-src -r -y

# MARKER="# ARENA-ROSNAV"
# SHELLS=(~/.zshrc ~/.bashrc)
# 
# # add to .<shell>rc if exists
# for SHELL in "${SHELLS[@]}"
# do
#   if [ -e "$SHELL" ]; then
#     if ! grep -q "$MARKER" "$SHELL"; then
#       echo "Adding to $SHELL"
#       echo '' >> "$SHELL"
#       echo "$MARKER" >> "$SHELL"
# #      echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$SHELL"
# #      echo '. "$(cd src/arena/arena-rosnav && poetry env info -p)/bin/activate"' >> "$SHELL"
#       echo 'source ${TARGET_DIR}/install/local_setup.bash' >> "$SHELL"
#       echo '' >> "$SHELL"
#     fi
#   fi
# done