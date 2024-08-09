FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV PATH="/root/.local/bin:/root/.pyenv/bin:$PATH"
ENV PYENV_ROOT="/root/.pyenv"
ENV ARENA_WS_DIR="/arena4_ws"

ARG ARENA_ROS_VERSION=jazzy
ARG ARENA_BRANCH=jazzy

# Set the timezone
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Add ROS 2 apt repository
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install a comprehensive set of libraries and tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    sudo \
    git \
    wget \
    libssl-dev \
    zlib1g-dev \
    libncurses5-dev \
    libgdbm-dev \
    libnss3-dev \
    libsqlite3-dev \
    libreadline-dev \
    libffi-dev \
    libbz2-dev \
    liblzma-dev \
    python3 \
    python3-pip \
    python3-dev \
    software-properties-common

# Download and run the install script
ADD install.sh install.sh
# ADD https://raw.githubusercontent.com/voshch/arena-rosnav/${ARENA_BRANCH}/installers/install.sh install.sh
RUN bash -c 'echo -e "\n\n\n\n\n\n\n" | bash install.sh'

CMD ["bash"]