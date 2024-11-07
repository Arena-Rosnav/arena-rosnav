#!/bin/bash -i
cd "${ARENA_WS_DIR}"

source $(cd src/arena/arena-rosnav && poetry env info -p)/bin/activate

#Optional choice: install a CUDA-enabled PyTorch 2.4.0 build based on the CUDA version available on your system
python -m pip install torch==2.4.0 --index-url https://download.pytorch.org/whl/cu121

#Ensure upgrade the latest pip version
python -m pip install --upgrade pip

#Install typegaurd dependencies
python -m pip install typeguard

#Install isaac sim
echo "Downloading Isaac sim ..."
python -m pip install isaacsim==4.2.0.2 --extra-index-url https://pypi.nvidia.com

#Install Isaac sim - python package
echo "Downloading Isaac sim - python package ..."
python -m pip install isaacsim-extscache-physics==4.2.0.2 isaacsim-extscache-kit==4.2.0.2 isaacsim-extscache-kit-sdk==4.2.0.2 --extra-index-url https://pypi.nvidia.com
python -m pip install isaacsim-kernel isaacsim-app isaacsim-asset isaacsim-benchmark isaacsim-code-editor isaacsim-core isaacsim-cortex isaacsim-example isaacsim-gui isaacsim-replicator isaacsim-rl isaacsim-robot isaacsim-robot-motion isaacsim-robot-setup isaacsim-ros1 isaacsim-ros2 isaacsim-sensor isaacsim-storage isaacsim-template isaacsim-test isaacsim-utils --extra-index-url https://pypi.nvidia.com

echo "Completed download Isaac sim"