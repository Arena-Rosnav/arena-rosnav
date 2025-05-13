#! /usr/bin/env bash

trap "trap - ERR; return $?;" ERR

if [ -z ${ARENA_SOURCED+x} ] ; then
    export ARENA_WS_DIR="$(pwd)"
    export ARENA_ROS_DISTRO=${ARENA_ROS_DISTRO:-humble}

    # Set Gazebo version if not provided
    export GAZEBO_VERSION=${GAZEBO_VERSION:-harmonic}
    export GZ_VERSION=${GAZEBO_VERSION}

    export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
    export ROS_DOMAIN_ID=1
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/opt/ros/${ARENA_ROS_DISTRO}/lib/"
    export INSTALLED=$ARENA_WS_DIR/src/arena/arena-rosnav/.installed
    
    # stop rviz from flashing
    export QT_SCREEN_SCALE_FACTORS=1

    pushd src/arena/arena-rosnav
        export VIRTUAL_ENV_DISABLE_PROMPT=1
        venv_path="$(poetry env info -p)"
        # if [ "$venv_path" != "$VIRTUAL_ENV" ] ; then
            source "$venv_path"/bin/activate
            dirs=("$venv_path"/lib/*/site-packages)
            _IFS=$IFS IFS=':'
            export PYTHONPATH="${dirs[*]}:$PYTHONPATH"
            IFS=$_IFS
            unset _IFS
            unset dirs
        unset venv_path
    popd

    r2st(){ ros2 service call "$1" "$(ros2 service type "$1")";}

    export ARENA_SOURCED=1
    export PS1="(arena) $PS1"
    echo 'sourced arena environment'
fi

if [ -f "/opt/ros/${ARENA_ROS_DISTRO}/setup.bash" ] ; then
    . "/opt/ros/${ARENA_ROS_DISTRO}/setup.bash"
fi
if [ -f install/local_setup.bash ] ; then
    . install/local_setup.bash
fi 

if [ -f "$INSTALLED" ] && grep -q "isaac.sh" "$INSTALLED"; then
    if [ -z ${ISAAC_PATH+x} ] ; then
        source ~/isaacsim-4.2.0/setup.bash
    fi
fi

trap - ERR
