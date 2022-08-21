## Simple Installation

all requirements see [here](https://arena-rosnav-wiki.readthedocs.io/en/latest/user_guides/installation/)

Install Conda
```
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh 
bash Miniconda3-latest-Linux-x86_64.sh
source ~/.bashrc
conda config --set auto_activate_base false 
```

Clone repo in any catkin ws or create new catkin ws

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

```
git clone https://github.com/Arena-Rosnav/arena-rosnav.git
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
Optional: To ensure the isolation of the python environment, run poetry inside conda environment is better

```
poetry config virtualenvs.create false //run without conda env
conda create -n rosnav python=3.8
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
cd ../../.. && catkin_make
```


Finished!

## Training

Open Terminal 1 (Run roslaunch)
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch arena_bringup start_arena_flatland.launch task_mode:="random"
```

Open Terminal 2 (Run Script)
```
cd ~/catkin_ws
source devel/setup.bash
cd src/utils/arena2d/rl-ros-agents/
conda env create -f environment.yml //do once
conda activate arena2d
python scripts/training/train_a3c.py
```

#### TODO

Everything
