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

Build catkin

```
cd ../../.. && catkin_make
```


Finished!

## Run Simulator

Open Terminal 1 (Run roslaunch)
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch ...
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
