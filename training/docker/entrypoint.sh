#!/bin/bash

# Start virtual X server in the background
# - DISPLAY default is :99, set in dockerfile
# - Users can override with `-e DISPLAY=` in `docker run` command to avoid
#   running Xvfb and attach their screen
if [[ -x "$(command -v Xvfb)" && "$DISPLAY" == ":99" ]]; then
    echo "Starting Xvfb"
    Xvfb :99 -screen 0 1600x1200x24+32 &
fi

# Check if the ROS_DISTRO is passed and use it
# to source the ROS environment
if [ -n "${ROS_DISTRO}" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

# Check if the shell is bash or zsh
if [[ -n "$SHELL" ]]; then
    shell=$(basename $SHELL)
fi

echo "[ARENA-ROSNAV] Using $shell as the shell."

# Set the workspace variable
arena_ws="${ARENA_WS:-catkin_ws}"
arena_root="${ARENA_ROOT:-$HOME}"

cd $arena_root/$arena_ws/src/arena/arena-rosnav/training/scripts

if [[ "$shell" == "bash" ]]; then
    bash train_agent.sh
else
    zsh train_agent.zsh
fi