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

mkdir -p src/gazebo
vcs import src/gazebo < "src/arena/arena-rosnav/.repos/gazebo.repos"

sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update

pushd src/gazebo
  sudo apt-get install -y \
    $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ') \
    || true
popd

rosdep install -r --from-paths src/gazebo -i -y --rosdistro ${ARENA_ROS_VERSION} \
  || echo 'rosdep failed to install all dependencies'

export GZ_VERSION=${GAZEBO_VERSION}