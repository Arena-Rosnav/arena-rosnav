## Simple Installation

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
roslaunch arena_bringup start_arena_flatland.launch
// roslaunch arena_bringup arena2d_simulator.launch 
// only for arena2d
```

Open Terminal 2 (Run Script)
```
poetry shell # Firstly load python virtual environment
cd ~/catkin_ws
source devel/setup.bash # To load other dependencies
cd <script directory>
python ...
```

#### TODO

Everything
