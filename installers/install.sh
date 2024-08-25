#!/bin/bash -i
set -e

export ARENA_ROSNAV_REPO=${ARENA_ROSNAV_REPO:-voshch/arena-rosnav}
export ARENA_BRANCH=${ARENA_BRANCH:-humble}
export ARENA_ROS_VERSION=${ARENA_ROS_VERSION:-humble}

# == read inputs ==
echo 'Configuring arena-rosnav...'

ARENA_WS_DIR=${ARENA_WS_DIR:-~/arena4_ws}
read -p "arena-rosnav workspace directory [${ARENA_WS_DIR}] " INPUT
export ARENA_WS_DIR=$(realpath "$(eval echo ${INPUT:-${ARENA_WS_DIR}})")

sudo echo "$ARENA_WS_DIR"

# == python deps ==

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
  source ~/.bashrc
fi
poetry config virtualenvs.in-project true

# == compile ros ==


sudo add-apt-repository universe -y
sudo apt-get update || echo 0
sudo apt-get install -y curl

echo "Installing tzdata...:"
export DEBIAN_FRONTEND=noninteractive
sudo apt install -y tzdata && sudo dpkg-reconfigure --frontend noninteractive tzdata

# ROS
echo "Setting up ROS2 ${ARENA_ROS_VERSION}..."

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


#python env
mkdir -p "${ARENA_WS_DIR}/src/arena/arena-rosnav"
cd "${ARENA_WS_DIR}/src/arena/arena-rosnav"
curl "https://raw.githubusercontent.com/${ARENA_ROSNAV_REPO}/${ARENA_BRANCH}/pyproject.toml" > pyproject.toml

mkdir -p "${ARENA_WS_DIR}/src/arena/arena-rosnav/tools"
cd "${ARENA_WS_DIR}/src/arena/arena-rosnav/tools"
curl "https://raw.githubusercontent.com/${ARENA_ROSNAV_REPO}/${ARENA_BRANCH}/tools/poetry_install" > poetry_install
curl "https://raw.githubusercontent.com/${ARENA_ROSNAV_REPO}/${ARENA_BRANCH}/tools/colcon_build" > colcon_build
cd "${ARENA_WS_DIR}" 
. "${ARENA_WS_DIR}/src/arena/arena-rosnav/tools/poetry_install"

# Getting Packages
echo "Installing Deps...:"

sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev \
    ros-dev-tools

# python -m pip install \
#     colcon-common-extensions \
#     flake8-blind-except \
#     flake8-class-newline \
#     flake8-deprecated \
#     mypy \
#     pip \
#     pytest \
#     pytest-cov \
#     pytest-mock \
#     pytest-repeat \
#     pytest-rerunfailures \
#     pytest-runner \
#     pytest-timeout

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

mkdir -p "${ARENA_WS_DIR}/src/ros2"
cd "${ARENA_WS_DIR}"
curl "https://raw.githubusercontent.com/ros2/ros2/${ARENA_ROS_VERSION}/ros2.repos" > ros2.repos
until vcs import src/ros2 < ros2.repos ; do echo "failed to update, retrying..." ; done
rosdep install --from-paths src --ignore-src --rosdistro ${ARENA_ROS_VERSION} -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

cd "${ARENA_WS_DIR}"
. "${ARENA_WS_DIR}/src/arena/arena-rosnav/tools/colcon_build"

# == install gazebo on top of ros2 ==
cd "${ARENA_WS_DIR}"

sudo apt-get install -y \
  python3-pip \
  lsb-release \
  gnupg \
  curl

curl -O https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-harmonic.yaml
mkdir -p "${ARENA_WS_DIR}/src/gazebo"
vcs import "${ARENA_WS_DIR}/src/gazebo" < collection-harmonic.yaml

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update

cd "${ARENA_WS_DIR}/src/gazebo"
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')

cd "${ARENA_WS_DIR}"
. "${ARENA_WS_DIR}/src/arena/arena-rosnav/tools/colcon_build"

# == install arena on top of ros2 ==

rm -r "${ARENA_WS_DIR}/src/arena/arena-rosnav"
echo "cloning Arena-Rosnav into ${ARENA_WS_DIR}..."
git clone --branch "${ARENA_BRANCH}" "https://github.com/${ARENA_ROSNAV_REPO}.git" "${ARENA_WS_DIR}/src/arena/arena-rosnav"

cd "${ARENA_WS_DIR}"
. "${ARENA_WS_DIR}/src/arena/arena-rosnav/tools/poetry_install"

cd "${ARENA_WS_DIR}"
until vcs import src < src/arena/arena-rosnav/arena.repos ; do echo "failed to update, retrying..." ; done

# == optinal installers ==

cd "${ARENA_WS_DIR}/src/arena/arena-rosnav/installers"

# install planner deps (optional)
read -p "Install all planners? [Y] " choice
choice="${choice:-Y}"
if [[ "$choice" =~ ^[Yy] ]]; then
    $SHELL planners.sh
fi

# install traininp deps (optional)
read -p "Install training dependencies? [N] " choice
choice="${choice:-N}"
if [[ "$choice" =~ ^[Yy] ]]; then
    $SHELL training.sh
fi

cd "${ARENA_WS_DIR}"
ln -s src/arena/arena-rosnav/tools/poetry_install .
ln -s src/arena/arena-rosnav/tools/colcon_build .

# final pass
rosdep install --from-paths src --ignore-src --rosdistro ${ARENA_ROS_VERSION} -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"
cd "${ARENA_WS_DIR}"
. "${ARENA_WS_DIR}/src/arena/arena-rosnav/tools/colcon_build"
