#!/bin/bash -i
set -e

echo 'Configuring arena-rosnav...'

# read inputs
ARENA_WS_DIR=${ARENA_WS_DIR:-"~/arena_ws"}
read -p "arena-rosnav workspace directory [${ARENA_WS_DIR}] " INPUT
export ARENA_WS_DIR="$(eval echo ${INPUT:-${ARENA_WS_DIR}})"

ARENA_ROS_VERSION=${ARENA_ROS_VERSION:-jazzy}
read -p "ros version [${ARENA_ROS_VERSION}] " INPUT
export ARENA_ROS_VERSION=${INPUT:-${ARENA_ROS_VERSION}}

ARENA_BRANCH=${ARENA_BRANCH:-master}
read -p "arena-rosnav branch [${ARENA_BRANCH}] " INPUT
export ARENA_BRANCH=${INPUT:-${ARENA_BRANCH}}

# # clone arena-rosnav
# if [[ -d ${ARENA_WS_DIR}/src/arena/arena-rosnav ]]; then
#   echo "updating existing arena-rosnav installation in ${ARENA_WS_DIR}..."
#   cd ${ARENA_WS_DIR}/src/arena/arena-rosnav
#   git pull https://github.com/jokrasa1011/arena-rosnav.git "${ARENA_BRANCH}"
# else
#   echo "cloning Arena-Rosnav into ${ARENA_WS_DIR}..."
#   git clone --branch "${ARENA_BRANCH}" https://github.com/jokrasa1011/arena-rosnav.git "${ARENA_WS_DIR}/src/arena/arena-rosnav"
# fi
cd "${ARENA_WS_DIR}"

# run installers
cd src/arena/arena-rosnav/setup

# install ros
cd distro
$SHELL install.sh
cd ..
source ~/.bashrc

# install arena-rosnav base
cd arena
$SHELL install.sh
cd ..
source ~/.bashrc

# install planner deps (optional)
read -p "Install all planners? [Y] " choice
choice="${choice:-Y}"
if [[ "$choice" =~ ^[Yy] ]]; then
    cd optional
    $SHELL install_planners.sh
    cd ..
fi

# install traininp deps (optional)
read -p "Install training dependencies? [N] " choice
choice="${choice:-N}"
if [[ "$choice" =~ ^[Yy] ]]; then
    cd optional
    $SHELL install_training.sh
    cd ..
fi

cd "${ARENA_WS_DIR}"
source "/opt/ros/${ARENA_ROS_VERSION}/setup.bash"
source $(cd src/arena/arena-rosnav/setup/arena && poetry env info -p)/bin/activate
colcon build  --symlink-install --cmake-args " -DPython3_ROOT_DIR=$(cd src/arena/arena-rosnav/setup/arena && poetry env info -p)"