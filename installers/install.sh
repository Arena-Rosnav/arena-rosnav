#!/bin/bash -i
set -e

export ARENA_ROSNAV_REPO=${ARENA_ROSNAV_REPO:-voshch/arena-rosnav}
export ARENA_BRANCH=${ARENA_BRANCH:-humble}
export ARENA_ROS_DISTRO=${ARENA_ROS_DISTRO:-humble}

# == read inputs ==
echo 'Configuring arena-rosnav...'

ARENA_WS_DIR=${ARENA_WS_DIR:-~/arena4_ws}
read -p "arena-rosnav workspace directory [${ARENA_WS_DIR}] " INPUT
export ARENA_WS_DIR=$(realpath "$(eval echo ${INPUT:-${ARENA_WS_DIR}})")

echo "installing ${ARENA_ROSNAV_REPO}:${ARENA_BRANCH} on ROS2 ${ARENA_ROS_DISTRO} to ${ARENA_WS_DIR}"
sudo echo 'confirmed'
mkdir -p "$ARENA_WS_DIR"
cd "$ARENA_WS_DIR"

export INSTALLED=src/arena/arena-rosnav/.installed

# == remove ros problems ==
files=$((grep -l "/ros" /etc/apt/sources.list.d/* | grep -v "ros2") || echo '')

if [ -n "$files" ]; then
    echo "The following files can cause some problems to installer:"
    echo "$files"
    read -p "Do you want to delete these files? (Y/n) [Y]: " choice
    choice=${choice:-Y}

    if [[ "$choice" == "y" || "$choice" == "Y" ]]; then
        sudo rm -f $files
        echo "Deleted $(echo $files)"
    fi
    unset choice
fi

# == python deps ==

# pyenv
if [ ! -d "$HOME/.pyenv" ] ; then
  rm -rf "$HOME/.pyenv"
  curl https://pyenv.run | bash
  echo 'export PYENV_ROOT="$HOME/.pyenv"'                                 >> ~/.bashrc
  echo '[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"'  >> ~/.bashrc
  echo 'eval "$(pyenv init -)"'                                           >> ~/.bashrc
  
  . ~/.bashrc

  # resourcing does not work in the same shell
  export PYENV_ROOT="$HOME/.pyenv"
  export PATH="$PYENV_ROOT/bin:$PATH"
  eval "$(pyenv init -)"

  which pyenv || (echo 'open a completely new shell'; exit 1)
fi

# Poetry
if ! which poetry ; then
  echo "Installing Poetry...:"
  curl -sSL https://install.python-poetry.org | python3 -
  if ! grep -q 'export PATH="$HOME/.local/bin"' ~/.bashrc; then
    echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
    source ~/.bashrc
  fi
  $HOME/.local/bin/poetry config virtualenvs.in-project true
fi

# == compile ros ==


sudo add-apt-repository universe -y
sudo apt-get update || echo 0
sudo apt-get install -y curl

echo "Installing tzdata...:"
export DEBIAN_FRONTEND=noninteractive
sudo apt-get install -y tzdata libompl-dev
sudo dpkg-reconfigure --frontend noninteractive tzdata

# ROS
echo "Setting up ROS2 ${ARENA_ROS_DISTRO}..."

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


# for building python
echo "Installing Python deps..." 
sudo apt-get install -y build-essential python3-pip zlib1g-dev libffi-dev libssl-dev libbz2-dev libreadline-dev libsqlite3-dev liblzma-dev libncurses-dev tk-dev python3-dev

if [ ! -d src/arena/arena-rosnav/tools ] ; then
  mkdir -p src/arena/arena-rosnav/tools
  pushd src/arena/arena-rosnav/tools
    curl "https://raw.githubusercontent.com/${ARENA_ROSNAV_REPO}/${ARENA_BRANCH}/tools/poetry_install" > poetry_install
    curl "https://raw.githubusercontent.com/${ARENA_ROSNAV_REPO}/${ARENA_BRANCH}/tools/colcon_build" > colcon_build
  popd
fi

if [ ! -f "${ARENA_WS_DIR}/src/arena/arena-rosnav/pyproject.toml" ] ; then
  #python env
  
  mkdir -p src/arena/arena-rosnav
  pushd src/arena/arena-rosnav
    curl "https://raw.githubusercontent.com/${ARENA_ROSNAV_REPO}/${ARENA_BRANCH}/pyproject.toml" > pyproject.toml
  popd
fi
. src/arena/arena-rosnav/tools/poetry_install

# vcstool fork
python -m pip install git+https://github.com/voshch/vcstool.git
alias vcs='$HOME/.pyenv/shims/vcs' # avoid reopening shell

# Getting Packages
echo "Installing deps...:"
sudo apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    libasio-dev \
    libtinyxml2-dev \
    libcunit1-dev \
    libpcl-dev \
    libboost-python-dev \
    python3-rosdep \
    libgps-dev \
    graphicsmagick \
    libgraphicsmagick1-dev \
    nlohmann-json3-dev \
    libxtensor-dev \
    libceres-dev \
    libsuitesparse-dev

# Check if the default ROS sources.list file already exists
ros_sources_list="/etc/ros/rosdep/sources.list.d/20-default.list"
if [[ -f "$ros_sources_list" ]]; then
  echo "rosdep appears to be already initialized"
  echo "Default ROS sources.list file already exists:"
  echo "$ros_sources_list"
else
  sudo rosdep init
fi

rosdep update --rosdistro "${ARENA_ROS_DISTRO}"

if [ ! -d src/deps ] ; then
  #TODO resolve this through vcstool
  mkdir -p src/deps
  pushd src/deps
    git clone --filter=tree:0 --depth 1 https://github.com/ros-perception/pcl_msgs.git -b ros2
    git clone --filter=tree:0 --depth 1 https://github.com/rudislabs/actuator_msgs.git
    git clone --filter=tree:0 --depth 1 https://github.com/swri-robotics/gps_umd.git -b ros2-devel
    git clone --filter=tree:0 --depth 1 https://github.com/ros-perception/vision_msgs.git -b ros2
    git clone --filter=tree:0 --depth 1 https://github.com/ros-perception/vision_opencv.git -b humble
  popd
fi

if [ ! -f src/ros2/compiled ] ; then
  # install ros2

  mkdir -p src/ros2
  curl "https://raw.githubusercontent.com/ros2/ros2/${ARENA_ROS_DISTRO}/ros2.repos" > ros2.repos
  vcs import src/ros2 < ros2.repos
  
  RTI_NC_LICENSE_ACCEPTED=yes \
  rosdep install \
    --from-paths src/ros2 \
    --ignore-src \
    --rosdistro "${ARENA_ROS_DISTRO}" \
    -y \
    || echo 'rosdep failed to install all dependencies'

  # fix rosidl error that was caused upstream https://github.com/ros2/rosidl/issues/822#issuecomment-2403368061
  pushd src/ros2/ros2/rosidl
    git -c user.name='Arena' -c user.email='anonymous@arena-rosnav.org' cherry-pick 654d6f5658b59009147b9fad9b724919633f38fe || echo 'already cherry picked'
  popd

  . src/arena/arena-rosnav/tools/colcon_build --paths src/ros2/* --packages-skip-build-finished
  touch src/ros2/compiled
  
  # don't even ask
  rm -rf build/foonathan_memory_vendor
fi

# == install arena on top of ros2 ==

if [ ! -f "$INSTALLED" ] ; then
  mv src/arena/arena-rosnav src/arena/.arena-rosnav

  echo "cloning Arena-Rosnav..."
  git clone --branch "${ARENA_BRANCH}" "https://github.com/${ARENA_ROSNAV_REPO}.git" src/arena/arena-rosnav

  mv -n src/arena/.arena-rosnav/* src/arena/.arena-rosnav/.* src/arena/arena-rosnav || true
  rm -rf src/arena/.arena-rosnav


  ln -fs src/arena/arena-rosnav/tools/source.bash ./arena.bash
  ln -fs src/arena/arena-rosnav/tools/poetry_install .
  ln -fs src/arena/arena-rosnav/tools/colcon_build .

  . poetry_install
fi

vcs import src < src/arena/arena-rosnav/arena.repos
rosdep install -y \
  --from-paths src \
  --ignore-src \
  --rosdistro "$ARENA_ROS_DISTRO" \
  || echo 'rosdep failed to install all dependencies'
. poetry_install
touch "$INSTALLED"

if [ ! -d /usr/local/include/lightsfm ] ; then
  git clone https://github.com/robotics-upo/lightsfm.git lightsfm
  (cd lightsfm && make && sudo make install || rm -rf lightsfm)
  rm -rf lightsfm || echo 'failed to install lightsfm'
fi

# run installers
# sudo apt upgrade

compile(){
  cd "${ARENA_WS_DIR}"
  . colcon_build --packages-skip-build-finished #TODO get rid of this
  ARENA_ROS_DISTRO=${ARENA_ROS_DISTRO} ros2 run arena_bringup pull
  . colcon_build --packages-skip-build-finished
}

compile

# robot dependencies

sudo apt install -y ros-${ROS_DISTRO}-irobot-create-description
sudo apt install -y ros-${ROS_DISTRO}-irobot-create-msgs

for installer in $(ls src/arena/arena-rosnav/installers | grep -E '^[0-9]+_.*.sh') ; do 

  name=$(echo $installer | cut -d '_' -f 2)

  if grep -q "$name" "$INSTALLED" ; then
    echo "$name already installed"
  else
    read -p "Do you want to install ${name}? [N] " choice
    choice="${choice:-N}"
    if [[ "$choice" =~ ^[Yy]$ ]]; then
        . "src/arena/arena-rosnav/installers/$installer"
        compile
        echo "$name" >> "$INSTALLED"
    else
        echo "Skipping ${name} installation."
    fi
    unset choice
  fi
done


# final pass
compile

echo 'installation finished'
