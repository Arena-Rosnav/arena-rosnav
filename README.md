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
cd ../../.. && catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_CXX_STANDARD=14
```


Finished!

#### TODO

Everything
