FROM ubuntu:22.04

ENV ARENA_DIR=/arena_ws
ENV BRANCH=ros2

ARG DEBIAN_FRONTEND=noninteractive

SHELL [ "/usr/bin/bash", "-c" ]

RUN apt update
RUN apt install -y sudo
RUN apt install -y tzdata && dpkg-reconfigure --frontend noninteractive tzdata
RUN apt install -y software-properties-common curl

#install
    # apt
        # add apt repos
            RUN add-apt-repository universe
            RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
                echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

        # update
            ADD arena-rosnav/setup /tmp/arena
            WORKDIR /tmp/arena
            RUN apt update
            RUN apt install -y $(awk '{print $1}' install1/package.list)
            WORKDIR /
            RUN rm -r /tmp/arena
            
    RUN rosdep init && rosdep update

    # poetry
        RUN curl -sSL https://install.python-poetry.org | python3 - && echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc

#install2
    WORKDIR ${ARENA_DIR}

    # sources
        #TEMP
        # build as: docker build -t arena-rosnav:humble -f arena/arena-rosnav/setup/Dockerfile arena
        ADD . src/arena
        #END TEMP
        
        # RUN git clone --depth=1 --branch ${BRANCH} https://github.com/Arena-Rosnav/arena-rosnav.git src/arena/arena-rosnav
        # RUN until vcs import src < src/arena/arena-rosnav/setup/.repos ; do echo "failed to update, retrying..." ; done

        # compat
        RUN ln -rs src/arena/arena-rosnav/setup/install2/* src/arena/arena-rosnav/

    # deps
        # poetry
            ENV PYTHON_KEYRING_BACKEND=keyring.backends.fail.Keyring
            RUN cd src/arena/arena-rosnav && \
                $HOME/.local/bin/poetry install || \
                ($HOME/.local/bin/poetry lock --no-update && $HOME/.local/bin/poetry install)

        # rosdep
            RUN rosdep update && apt update
            RUN rosdep install --from-paths src --ignore-src -r -y
            RUN apt install -y $(awk '{print $1}' src/arena/arena-rosnav/setup/install2/package.list)
            
    # build and export
#        RUN source "$(cd src/arena/arena-rosnav && poetry env info -p)/bin/activate" && \
#            source /opt/ros/humble/setup.bash && \
#            colcon build --symlink-install --cmake-args " -DPython3_ROOT_DIR=$(cd src/arena/arena-rosnav && poetry env info -p)"
#
#        RUN echo "source $(pwd)/install/local_setup.bash" >> ~/.bashrc