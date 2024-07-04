#!/bin/bash -i

TARGET_DIR=${1:-~/arena_ws}
branch=${2:-ros2}

set -e
 
# Store the current working directory
current_dir="$(pwd)"

sudo echo ""
 
# Project Setup
echo "Preparing Project...:"
mkdir -p ${TARGET_DIR}
cd ${TARGET_DIR}

# clone arena-rosnav
if [[ -d ${TARGET_DIR}/src/arena/arena-rosnav ]]; then
  cd ${TARGET_DIR}/src/arena/arena-rosnav
  git pull https://github.com/jokrasa1011/arena-rosnav.git ${branch}
  cd ${TARGET_DIR}
else
  git clone --branch ${branch} https://github.com/jokrasa1011/arena-rosnav.git src/arena/arena-rosnav
fi

until vcs import src < src/arena/arena-rosnav/.repos ; do echo "failed to update, retrying..." ; done
#

#compat
ln -rs src/arena/arena-rosnav/setup/install2/* src/arena/arena-rosnav/
#python env init
cd src/arena/arena-rosnav
export PYTHON_KEYRING_BACKEND=keyring.backends.fail.Keyring 
$HOME/.local/bin/poetry install || \
                ($HOME/.local/bin/poetry lock --no-update && $HOME/.local/bin/poetry install)
#
 cd ${TARGET_DIR}
# Missing Deps
echo "Installing Missing Deps...:"

rosdep update && rosdep install --from-paths src --ignore-src -r -y
apt install -y $(awk '{print $1}' src/arena/arena-rosnav/setup/install2/package.list)

# Project Install
echo "Installing Project...:"

exit 12
#TODO

colcon build

 

MARKER="# ARENA-ROSNAV"
SHELLS=(~/.zshrc ~/.bashrc)

# add to .<shell>rc if exists
for SHELL in "${SHELLS[@]}"
do
  if [ -e "$SHELL" ]; then
    if ! grep -q "$MARKER" "$SHELL"; then
      echo "Adding to $SHELL"
      echo '' >> "$SHELL"
      echo "$MARKER" >> "$SHELL"
#      echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$SHELL"
#      echo '. "$(cd src/arena/arena-rosnav && poetry env info -p)/bin/activate"' >> "$SHELL"
      echo 'source ${TARGET_DIR}/install/local_setup.bash' >> "$SHELL"
      echo '' >> "$SHELL"
    fi
  fi
done
 
# Return to the original working directory
cd "$current_dir"
 
echo ""
echo "Installation Complete."
echo "You can confirm that it works, by running the following command in a NEW terminal:"
echo ""
echo "roslaunch arena_bringup start_arena.launch"
echo ""
echo "If you need to train or use rosnav/aio planners, download and run install3_training.sh."