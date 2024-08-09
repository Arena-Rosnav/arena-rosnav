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

# Download and run the install script
ADD install.sh install.sh
# ADD https://raw.githubusercontent.com/voshch/arena-rosnav/${ARENA_BRANCH}/installers/install.sh install.sh
RUN bash -c 'echo -e "\n\n\n\n\n\n\n" | bash install.sh'

CMD ["bash"]