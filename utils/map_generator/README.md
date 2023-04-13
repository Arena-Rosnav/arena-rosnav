# MAP-GENERATOR

- Factory Design Pattern - modular design for easy implementation of further map algorithms
  - e.g. Rosnav, Barn
- Add the decorator "MapGeneratorFactory.register()"

```python
@MapGeneratorFactory.register("rosnav")
```

- Configuration file at: "_arena-rosnav/arena_bringup/params/map_generator.yaml_"

## MECHANISM

| Line | Publisher        | on topic                   | Subscriber       |
| ---- | ---------------- | -------------------------- | ---------------- |
| 1    | TaskMode         | "/request_new_map"         | MapGeneratorNode |
| 2    | MapGeneratorNode | "/map"                     | FlatlandNode     |
| 3    | MapGeneratorNode | "/map"                     | MapDistanceNode  |
| 4    | MapDistanceNode  | "/signal_new_distance_map" | TaskMode         |
| 5    | TaskMode         | "/dynamic_map/task_reset"  | TaskMode         |

Class | on topic | Class

### 1. TaskMode | "/request_new_map" | MapGeneratorNode

- When the maximum number of episodes per map is reached, the TaskMode class sends a "/request_new_map" message to the MapGeneratorNode class to request a new map on reset.

### 2. MapGeneratorNode | "/map" | FlatlandNode

- The MapGeneratorNode class generates a grid map using a specified algorithm, saves it as a PNG file in the "dynamic_map" folder, and sends it to the FlatlandNode class, updating the underlying (static) map.

### 3. MapGeneratorNode | "/map" | MapDistanceNode

- The MapGeneratorNode class triggers the replacement of the old distance map with the new map in the MapDistanceNode class when a new map is generated.

### 4. MapDistanceNode | "/signal_new_distance_map" | TaskMode

- The MapDistanceNode class sends a signal to continue resetting the scene and the TaskMode class receives it, with the distance map being crucial for the position generation of the robot and obstacles.

### 5. TaskMode | "/dynamic_map/task_reset" | TaskMode

- The TaskMode class sends a signal to all tasks in order to reset the scene, and the receiving end updates the MapManager with the recently generated distance map.

## NEW TASK MODES

### DM_RANDOM

### DM_STAGED
