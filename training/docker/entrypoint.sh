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

# Use the LOCAL_USER_ID if passed in at runtime
if [ -n "${LOCAL_USER_ID}" ]; then
    echo "Starting with UID : $LOCAL_USER_ID"
    # modify existing user's id
    usermod -u $LOCAL_USER_ID user
    # run as user
    exec gosu user "$@"
else
    exec "$@"
fi

# Check if the shell is bash or zsh
if [[ -n "$SHELL" ]]; then
    shell=$(basename $SHELL)
fi

echo "[ARENA-ROSNAV] Using $shell as the shell."

# Set the workspace variable
arena_ws="${ARENA_WS:-catkin_ws}"
arena_root="${ARENA_ROOT:-$HOME}"

cd $arena_root/$arena_root/src/arena/arena-rosnav/scripts
./train_agent.sh
