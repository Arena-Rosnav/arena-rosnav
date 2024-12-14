#!/bin/bash -i
cd "${ARENA_WS_DIR}"

source $(cd src/arena/arena-rosnav && poetry env info -p)/bin/activate

echo "Checking installed nvidia driver"
if ! command -v nvidia-smi &> /dev/null; then
    echo "Warning: nvidia-smi command not found. Terminating script."
    exit 1
fi

echo "nvidia-driver was installed"

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

touch ~/.ros/fastdds.xml
echo '<?xml version="1.0" encoding="UTF-8" ?>

<license>Copyright (c) 2022-2024, NVIDIA CORPORATION.  All rights reserved.
NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto.  Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.</license>


<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UdpTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="udp_transport_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>UdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>' > ~/.ros/fastdds.xml

SETUP_FILE = "~/.local/share/ov/pkg/isaac-sim-4.2.0/setup.bash"
# Write the content to the file
cat << 'EOF' > "$SETUP_FILE"
#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
MY_DIR="$(realpath -s "$SCRIPT_DIR")"
export CARB_APP_PATH=$SCRIPT_DIR/kit
export EXP_PATH=$MY_DIR/apps
export ISAAC_PATH=$MY_DIR
. ${MY_DIR}/setup_python_env.sh
EOF

echo "Completed download Isaac sim" 