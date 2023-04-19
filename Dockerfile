# Main Dockerfile

FROM osrf/ros:noetic-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get -y update \
    && apt-get install -y \
    apt-utils \
    software-properties-common \
    git \
    wget \
    ros-noetic-tf2 \
    ros-noetic-tf \
    ros-noetic-tf2-geometry-msgs \
    ffmpeg \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential \
    libsm6 \
    libxext6  \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get -y update \
    && apt-get -y install \
    libopencv-dev \
    liblua5.2-dev \
    screen \
    python3 \
    python3-dev \
    libpython3-dev \
    python3-catkin-pkg-modules \
    python3-rospkg-modules \
    python3-empy \
    python3-setuptools \
    ros-noetic-navigation \
    ros-noetic-teb-local-planner \
    ros-noetic-mpc-local-planner \
    libarmadillo-dev \
    ros-noetic-nlopt \
    python3 \
    python3-pip \
    tk \
    ros-noetic-turtlebot3-description \
    ros-noetic-turtlebot3-navigation \
    python-tk \
    python3-tk \
    tk-dev \
    ros-noetic-lms1xx \
    ros-noetic-velodyne-description \ 
    ros-noetic-hector-gazebo \
    ros-noetic-ira-laser-tools

#   Install Poetry
RUN pip3 install poetry \
    && pip3 install --upgrade pip

#   Install PyEnv
WORKDIR /root/
RUN git clone --depth=1 https://github.com/pyenv/pyenv.git .pyenv
ENV PYENV_ROOT="/root/.pyenv"
ENV PATH="${PYENV_ROOT}/shims:${PYENV_ROOT}/bin:${PATH}"

RUN echo 'eval "$(pyenv init -)"' >> /root/.bashrc
RUN sed -Ei -e '/^([^#]|$)/ {a export PYENV_ROOT="$HOME/.pyenv" \nexport PATH="$PYENV_ROOT/bin:$PATH"' -e ':a' -e '$!{n;ba};}' /root/.profile
RUN echo 'eval "$(pyenv init --path)"' >> /root/.profile

RUN mkdir -p /root/src/
WORKDIR /root/src/
COPY . /root/src/arena-rosnav

RUN echo -e "source /opt/ros/noetic/setup.sh" >> /root/.bashrc
RUN echo -e "source /root/devel/setup.sh" >> /root/.bashrc

RUN pip install scipy PyQt5 empy defusedxml 
RUN pip install wandb

WORKDIR /root/src/arena-rosnav
RUN rosws update

WORKDIR /root/
RUN source /root/.bashrc \
    && source /opt/ros/noetic/setup.sh \
    && catkin_make

WORKDIR /root/src/utils/stable-baselines3
RUN pip install -e .

WORKDIR /root/