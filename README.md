## Simple Installation

Follow [this](https://arena-rosnav-wiki.readthedocs.io/en/latest/user_guides/installation/) until step.7

Install Conda and Poetry 
```
// conda
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh 
bash Miniconda3-latest-Linux-x86_64.sh
source ~/.bashrc
conda config --set auto_activate_base false
conda create -n rosnav python=3.8

// poetry
curl -sSL https://install.python-poetry.org | python3 -
echo 'export PATH="$HOME/.local/bin:$PATH"'>> ~/.bashrc
source ~/.bashrc
poetry config virtualenvs.create false

//activate conda 
conda activate rosnav
```

Clone repo in any catkin ws or create new catkin ws

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

```
git clone https://github.com/zenghjian/arena-rosnav
```

Change into dir

```
cd arena-rosnav
```

Ros install

```
rosws update
```

Install python pkgs, you need poetry for this

```
poetry shell && poetry install
```

Install stable baselines

```
cd ../utils/stable-baselines3 && pip install -e .
```

Install arena2d requirements
```
sudo apt-get install cmake libsdl2-dev libfreetype-dev
```

Build catkin

```
cd ../../.. && catkin_make -DUSE_ROS=ON
```


Finished!

## Run Simulator

Open Terminal 1 (Run roslaunch)
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch arena_bringup start_arena.launch
// roslaunch arena_bringup arena2d_simulator.launch 
// only for arena2d
```

Open Terminal 2 (Run Script)
```
cd ~/catkin_ws
source devel/setup.bash 
cd src/arena-rosnav/
conda activate rosnav
poetry shell
cd ../utils/arena2d/rl-ros-agents/
python scripts/training/train_ppo.py
```

#### TODO

Everything

## Tipps for GPU 30X: m_86 is not compatible with the current PyTorch
```
conda rosnav
poetry shell
pip uninstall torch
pip list | grep torch
conda list | grep torch
pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu113
```
