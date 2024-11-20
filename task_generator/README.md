# Arena Rosnav Task Generator

This is the task generator package designed to work with the [Arena Benchmark](https://github.com/Arena-Rosnav/arena-rosnav) infrastructure.

## Task Modes

Our task generator package offers four task modes. We define a task as the process of a robot driving to a desired goal. A new task is started when the robot reaches the goal or after 3 min have past. For the scenario task, the task is only resetted for the in the scenario file defined amount.

### Random Task

Creates random static and dynamic obstacles when a new task is started. When starting the task a random goal and start position is selected. After the robot reaching the goal a new task is started.

### Staged Task

The staged task mode is designed for the trainings process of arena-rosnav. In general, it behaves like the random task mode but there are multiple stages between one can switch. Between the stages, the amount of static and dynamic obstacles changes. The amount of obstacles is defined in a curriculum file, the path to said file is a key in the `paths` parameter.

The **curriculum** file has the following schema.

```yaml
1:
  static: <amount of static obstacles for stage 1>
  dynamic: <amount of dynamic obstacles for stage 1>
2:
  static: <amount of static obstacles for stage 2>
  dynamic: <amount of dynamic obstacles for stage 2>
---
N:
  static: <amount of static obstacles for stage N>
  dynamic: <amount of dynamic obstacles for stage N>
```

### Scenario Task

The scenario task is especially designed for evaluation. One can defined scenarios
is a scenario file, which is read in when starting the simulation. In the scenario
file dynamic and static obstacles as well as the start and goal position of the
robot are defined.

The scenario declaration file can be created with [arena-tools](https://github.com/Arena-Rosnav/arena-tools) and has to follow
the specified file schema.

## Simulator Factory

To be able to use the task generator module in all our Simulators without changes, a unified interface between Simulator and task generator is needed. The interface contains a lot of functions to spawn, publish or move robots or obstacles, and a lot more.

At the moment we provide simulator interfaces for **Flatland** and **Gazebo**. In order to add a new simulator, in which the task generator should be used, a new simulator interface has to be created in `/taks_generator/simulators/` and has to be registrated in the simulator factory.

Your newly created simulator interface should derive the **BaseSimulator** located [here](TODO) and implement all functions. A detailed description of the functions is contained in the **BaseSimulator** itself.
