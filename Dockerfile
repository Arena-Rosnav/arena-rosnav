FROM mzahana/base-ubuntu20-cuda11.4.2:latest

ARG FROM_LOCAL=false
ARG ARENA_BRANCH=observation_refactor
ARG ARENA_ROOT=/root
ARG ARENA_WS=arena_ws

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

SHELL ["/bin/bash", "-c"]

RUN wget --quiet http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
    software-properties-common \
    apt-utils \
    ant \
    binutils \
    bc \
    net-tools \
    bash-completion \
    dirmngr \
    gazebo11 \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    libeigen3-dev \
    libgazebo11-dev \
    libgstreamer-plugins-base1.0-dev \
    libimage-exiftool-perl \
    libopencv-dev \
    libxml2-utils \
    mesa-utils \
    protobuf-compiler \
    x-window-system \
    ignition-edifice \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

# Some QT-Apps/Gazebo don't not show controls without this
ENV QT_X11_NO_MITSHM 1

ENV ROS_DISTRO noetic

# setup ros keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' \
    && sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-shadow.list' \
    && apt-get update \
    && apt-get -y --quiet --no-install-recommends install \
    geographiclib-tools \
    libeigen3-dev \
    libgeographic-dev \
    libopencv-dev \
    libyaml-cpp-dev \
    python3-rosdep \
    python3-catkin-tools \
    python3-catkin-lint \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-mavlink \
    ros-$ROS_DISTRO-mavros \
    ros-$ROS_DISTRO-mavros-extras \
    ros-$ROS_DISTRO-octomap \
    ros-$ROS_DISTRO-octomap-msgs \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-pcl-msgs \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-rostest \
    ros-$ROS_DISTRO-rosunit \
    ros-$ROS_DISTRO-tf-conversions \
    ros-$ROS_DISTRO-rqt-tf-tree \
    ros-$ROS_DISTRO-rviz \
    xvfb \
    && geographiclib-get-geoids egm96-5 \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

RUN pip3 install -U \
    osrf-pycommon

# bootstrap rosdep
RUN rosdep init && rosdep update

# Install dependencies
RUN echo "Installing Deps...:" && \
    apt-get install -y python3 python-is-python3 git python3-rosdep python3-pip python3-rosinstall-generator python3-vcstool build-essential python3-catkin-tools

# Auto completion in terminal
RUN apt install bash-completion
RUN echo "source /etc/profile.d/bash_completion.sh" >> $HOME/.bashrc

# Install poetry separated from system interpreter
RUN curl -sSL https://install.python-poetry.org | python3 -
ENV PATH "$ARENA_ROOT/.local/bin:$PATH"

# Install Arena-Rosnav
RUN if [ -d $ARENA_ROOT/$ARENA_WS/src/arena ]; then \
    echo \"Install Folder $ARENA_ROOT/arena_ws/src/arena/arena-rosnav already exists.\" && \
    echo \"This indicates Arena Rosnav is already installed.\" && \
    echo \"If you wish to reinstall, please delete $ARENA_ROOT/arena_ws\" && \
    exit 1; \
    fi 

WORKDIR $ARENA_ROOT/$ARENA_WS
ADD . src/arena/arena-rosnav

# Clone Arena-Rosnav
RUN if [ "$FROM_LOCAL" = "false" ]; then \
    rm -rf src/arena/arena-rosnav && \
    git clone --branch $ARENA_BRANCH https://github.com/Arena-Rosnav/arena-rosnav.git src/arena/arena-rosnav; \
    fi

RUN until vcs import src < src/arena/arena-rosnav/.repos ; do echo "failed to update, retrying..." ; done

# Setup and activate Poetry Env
WORKDIR $ARENA_ROOT/$ARENA_WS/src/arena/arena-rosnav
RUN poetry config virtualenvs.create true && \
    poetry install --no-root --no-interaction --no-ansi --with training && \
    poetry env use python3.9

WORKDIR $ARENA_ROOT/$ARENA_WS
# Install necessary dependencies
RUN apt-get update && apt-get install -y libopencv-dev liblua5.2-dev libarmadillo-dev ros-noetic-nlopt liblcm-dev psmisc
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Build Arena-Rosnav
RUN rm -r src/arena/utils/pedsim_ros/pedsim_engine/2ndparty/spencer_tracking_rviz_plugin
# RUN source /opt/ros/$ROS_DISTRO/setup.sh && catkin_make -DPYTHON_EXECUTABLE=$POETRY_ENV

WORKDIR $ARENA_ROOT/$ARENA_WS/src/arena/arena-rosnav/training/docker/
RUN chmod +x entrypoint.sh
ENTRYPOINT ./entrypoint.sh