#!/bin/bsh
echo "[ARENA-ROSNAV] Starting training shell script..."

# Check if the shell is bash or zsh
if [[ -n "$SHELL" ]]; then
    shell=$(basename $SHELL)
else
    echo "[ARENA-ROSNAV] Unsupported shell. Please use bash or zsh."
    exit 1
fi

# Set the workspace variable
arena_ws="${ARENA_WS:-catkin_ws}"
arena_root="${ARENA_ROOT:-$HOME}"

# Set the arena directory
arena_dir=${arena_root}/${arena_ws}/src/arena/arena-rosnav

echo "[ARENA-ROSNAV] Using workspace at: ${arena_dir}"

# Source ROS
source /opt/ros/noetic/setup.${shell}
# Source workspace
source ${arena_root}/${arena_ws}/devel/setup.${shell}
echo "[ARENA-ROSNAV] Sourced the workspace."

# Activate the poetry environment
POETRY_ENV_PATH=$(poetry env info --path)
echo "[ARENA-ROSNAV] Activated the poetry environment at $POETRY_ENV_PATH"

# Load config
source $arena_dir/arena_bringup/configs/training/train_from_sh.cfg

# Start training
roslaunch arena_bringup start_training.launch \
    num_envs:=$num_envs \
    model:=$model \
    map_folder_name:=$map_folder_name \
    entity_manager:=$entity_manager \
    sfm:=$sfm \
    cfg:=$cfg
