#!/bin/bash -i
set -e

# Set Gazebo version if not provided
export GAZEBO_VERSION=${GAZEBO_VERSION:-harmonic}

# Define Arena workspace directory
cd "${ARENA_WS_DIR}"

# Install dependencies
sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  lsb-release \
  gnupg \
  curl \
  libgps-dev

# Add OSRF Gazebo packages repository
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Add ROS 2 repository
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > \
  /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update package lists
sudo apt-get update

# Install Gazebo binaries and ROS-Gazebo bridge
sudo apt-get install -y \
  "gz-${GAZEBO_VERSION}" \
  libsdformat14-dev


export GZ_VERSION=${GAZEBO_VERSION}
mkdir -p "${ARENA_WS_DIR}/src/gazebo"

pushd "${ARENA_WS_DIR}/src/gazebo"
  if [ ! -d ros_gz ] ; then
    git clone https://github.com/voshch/ros_gz.git -b humble
  fi
popd

rosdep install -r --from-paths src -i -y --rosdistro "${ARENA_ROS_DISTRO}" 


echo "Gazebo ${GAZEBO_VERSION} and ROS-Gazebo bridge installed successfully!"

  
if [ ! -d tools/OpenUSD ]; then
  echo "Installing OpenUSD"
  mkdir -p tools
  pushd tools
    git clone --depth 1 -b v24.08 https://github.com/PixarAnimationStudios/OpenUSD.git
    sudo apt-get install -y libpyside2-dev python3-opengl cmake libglu1-mesa-dev freeglut3-dev mesa-common-dev
    USD_PATH="$(pwd)/OpenUSD/install"
    export USD_PATH
    cd OpenUSD
    python3 build_scripts/build_usd.py --build-variant release --no-tests --no-examples --no-imaging --onetbb --no-tutorials --no-docs --no-python "$USD_PATH"
    export PATH=$USD_PATH/bin:$PATH
    export LD_LIBRARY_PATH=$USD_PATH/lib:$LD_LIBRARY_PATH
    export CMAKE_PREFIX_PATH=$USD_PATH:$CMAKE_PREFIX_PATH
  popd
fi

if [ ! -d src/tools/gz-usd ]; then
  echo "Installing gz-usd"
  mkdir -p src/tools
  pushd src/tools
    git clone -b main https://github.com/gazebosim/gz-usd
  popd

  echo "Successfully installed gz-usd"
fi
