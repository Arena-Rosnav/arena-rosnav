#! /usr/bin/env bash

trap "trap - ERR; return $?;" ERR

if [ -z ${ARENA_SOURCED+x} ] ; then
    export ARENA_WS_DIR="$(pwd)"
    export ARENA_ROS_DISTRO=${ARENA_ROS_DISTRO:-humble}

    export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
    export ROS_DOMAIN_ID=1
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/lib/

    pushd src/arena/arena-rosnav
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
    export ARENA_SOURCED=1
    echo 'sourced arena environment'
fi

if [ -f "/opt/ros/${ARENA_ROS_DISTRO}/setup.bash" ] ; then
    . "/opt/ros/${ARENA_ROS_DISTRO}/setup.bash"
fi
if [ -f install/local_setup.bash ] ; then
    . install/local_setup.bash
fi 

if [ -f src/arena/arena-rosnav/isaac_setup.bash ] ; then
    if [ -z ${ISAAC_PATH+x} ] ; then
        . src/arena/arena-rosnav/isaac_setup.bash
    fi
fi

trap - ERR