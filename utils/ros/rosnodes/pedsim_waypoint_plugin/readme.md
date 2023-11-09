Don't forget to re-run `poetry install` in src/arena-rosnav.

Test with following commands. Change simulator to `gazebo` for 3D.

### pysocial1 ([PySocialForce](https://github.com/yuxiang-gao/PySocialForce))

```sh
roslaunch arena_bringup start_arena.launch simulator:=flatland task_mode:=scenario map_file:=map_empty model:=jackal pedsim_waypoint_plugin:=pysocial1
```

### pysocial2 ([PySocialForce](https://github.com/yuxiang-gao/PySocialForce))

```sh
roslaunch arena_bringup start_arena.launch simulator:=flatland task_mode:=scenario map_file:=map_empty model:=jackal pedsim_waypoint_plugin:=pysocial2
```

### deepsocialforce ([DeepSocialForce](https://github.com/svenkreiss/socialforce))


```sh
roslaunch arena_bringup start_arena.launch simulator:=flatland task_mode:=scenario map_file:=map_empty model:=jackal pedsim_waypoint_plugin:=deepsocialforce
```

### evacuation ([Evacuation-Bottleneck](https://github.com/fschur/Evacuation-Bottleneck))


```sh
roslaunch arena_bringup start_arena.launch simulator:=flatland task_mode:=scenario map_file:=map_empty model:=jackal pedsim_waypoint_plugin:=evacuation
```