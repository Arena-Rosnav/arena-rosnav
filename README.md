# For Unity Integration
unity-ros package needs to be installed. Thus, run rosws update and compile with catkin_make. 

## Simple Installation

Clone repo in any catkin ws or create new catkin ws

```
git clone git@github.com:Arena-Rosnav/arena-rosnav.git
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

#### TODO

Everything
