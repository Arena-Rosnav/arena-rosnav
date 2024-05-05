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
else
    echo "[ARENA-ROSNAV] Unsupported shell. Please use bash or zsh."
    exit 1
fi

echo "[ARENA-ROSNAV] Using $shell as the shell."

# Set the workspace variable
arena_ws="${ARENA_WS:-catkin_ws}"
arena_root="${ARENA_ROOT:-$HOME}"
arena_dir=${arena_root}/${arena_ws}/src/arena/arena-rosnav

if [[ "$shell" == "bash" ]]; then
    source /opt/ros/noetic/setup.bash
else
    source /opt/ros/noetic/setup.zsh
fi

# Retrieve the poetry environment path
cd $arena_dir && POETRY_ENV_PATH=$(poetry env info --path)/bin/python
# Build the workspace
cd $arena_root/$arena_ws && catkin_make -DPYTHON_EXECUTABLE=$POETRY_ENV_PATH

cd $arena_root/$arena_ws/src/arena/arena-rosnav/training/scripts
if [[ "$shell" == "bash" ]]; then
    bash train_agent.sh
else
    zsh train_agent.zsh
fi