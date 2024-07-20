#!/bin/bash -i
set -e
 
PYTHON_VERSION=3.10

sudo echo ""
 
# Project Setup
echo "Preparing Project...:"
cd ${ARENA_WS_DIR}

until vcs import src < src/arena/arena-rosnav/.repos ; do echo "failed to update, retrying..." ; done
#

#compat
ln -rfs src/arena/arena-rosnav/setup/install2/* src/arena/arena-rosnav/

#python env
pyenv install -s "${PYTHON_VERSION}"
pyenv local "${PYTHON_VERSION}"
cd src/arena/arena-rosnav/setup/arena
$HOME/.local/bin/poetry env use "${PYTHON_VERSION}"
export PYTHON_KEYRING_BACKEND=keyring.backends.fail.Keyring 
$HOME/.local/bin/poetry install || ($HOME/.local/bin/poetry lock --no-update && $HOME/.local/bin/poetry install)

# Missing Deps
sudo apt install -y $(cat package.list)

cd ${ARENA_WS_DIR}
echo "Installing Missing Deps...:"
rosdep update && rosdep install --from-paths src --ignore-src -r -y

# MARKER="# ARENA-ROSNAV"
# SHELLS=(~/.zshrc ~/.bashrc)
# 
# # add to .<shell>rc if exists
# for SHELL in "${SHELLS[@]}"
# do
#   if [ -e "$SHELL" ]; then
#     if ! grep -q "$MARKER" "$SHELL"; then
#       echo "Adding to $SHELL"
#       echo '' >> "$SHELL"
#       echo "$MARKER" >> "$SHELL"
# #      echo 'export PATH="$HOME/.local/bin:$PATH"' >> "$SHELL"
# #      echo '. "$(cd src/arena/arena-rosnav && poetry env info -p)/bin/activate"' >> "$SHELL"
#       echo 'source ${TARGET_DIR}/install/local_setup.bash' >> "$SHELL"
#       echo '' >> "$SHELL"
#     fi
#   fi
# done