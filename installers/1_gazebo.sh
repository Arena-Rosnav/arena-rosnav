#!/bin/bash -i
set -e

export GAZEBO_VERSION=${GAZEBO_VERSION:-garden}

# == install gazebo on top of ros2 ==
cd "${ARENA_WS_DIR}"

sudo apt-get install -y \
  python3-pip \
  lsb-release \
  gnupg \
  curl \
  libgps-dev

curl -O "https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-${GAZEBO_VERSION}.yaml"
mkdir -p "${ARENA_WS_DIR}/src/gazebo"
until vcs import "${ARENA_WS_DIR}/src/gazebo" < "collection-${GAZEBO_VERSION}.yaml" ; do echo "failed to update, retrying..." ; done


sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update

cd "${ARENA_WS_DIR}/src/gazebo"
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')
cd "${ARENA_WS_DIR}"

rosdep install -r --from-paths src/gazebo -i -y --rosdistro ${ARENA_ROS_VERSION}

# Install ros_gz
echo "Building ros_gz from source..."
export GZ_VERSION=${GAZEBO_VERSION}

cd "${ARENA_WS_DIR}/src"
git clone https://github.com/gazebosim/ros_gz.git -b ${ARENA_ROS_VERSION}
cd "${ARENA_WS_DIR}"