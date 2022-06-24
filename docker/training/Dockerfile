FROM arena-rosnav:latest

SHELL ["/bin/bash", "-c"]

ADD training_rosinstall /root/src/arena-rosnav/.rosinstall

WORKDIR /root/src/arena-rosnav
RUN rosws update

WORKDIR /root/
RUN source /root/.bashrc \
    && source /opt/ros/noetic/setup.sh \
    && catkin_make

ADD entry.sh /root/src/arena-rosnav/entry.sh
RUN chmod +x /root/src/arena-rosnav/entry.sh