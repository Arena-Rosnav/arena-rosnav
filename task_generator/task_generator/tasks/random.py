import random
from typing import List


from task_generator.constants import Constants
from task_generator.tasks.task_factory import TaskFactory

from task_generator.shared import DynamicObstacle, Obstacle, Waypoint

from task_generator.tasks.base_task import CreateObstacleTask


dynamic_obstacles_random = random.randint(
    Constants.Random.MIN_DYNAMIC_OBS, Constants.Random.MAX_DYNAMIC_OBS)
static_obstacles_random = random.randint(
    Constants.Random.MIN_STATIC_OBS, Constants.Random.MAX_STATIC_OBS)
interactive_obstacles_random = random.randint(
    Constants.Random.MIN_INTERACTIVE_OBS, Constants.Random.MAX_INTERACTIVE_OBS)


@TaskFactory.register(Constants.TaskMode.RANDOM)
class RandomTask(CreateObstacleTask):
    """
        The random task spawns static and dynamic
        obstacles on every reset and will create
        a new robot start and goal position for
        each task.
    """

    def reset(self, static_obstacles: int = 0, dynamic_obstacles: int = 0):
        return super().reset(
            lambda: self._reset_robot_and_obstacles(
                static_obstacles=static_obstacles,
                dynamic_obstacles=dynamic_obstacles,
            )
        )

    def _reset_robot_and_obstacles(self, dynamic_obstacles: int = 0, static_obstacles: int = 0):
        robot_positions: List[Waypoint] = []  # may be needed in the future idk

        interactive_obstacles: int = 0

        for manager in self._robot_managers:

            start_pos = self._map_manager.get_random_pos_on_map(
                manager.safe_distance)
            goal_pos = self._map_manager.get_random_pos_on_map(
                manager.safe_distance, forbidden_zones=[start_pos])

            manager.reset(start_pos=start_pos[:2], goal_pos=goal_pos[:2])

            robot_positions.append(start_pos)
            robot_positions.append(goal_pos)

        self._obstacle_manager.reset()
        self._map_manager.init_forbidden_zones()

        dynamic_obstacles_array: List[DynamicObstacle]
        static_obstacles_array: List[Obstacle]
        interactive_obstacles_array: List[Obstacle]

        self._obstacle_manager.spawn_map_obstacles()

        # Create static obstacles
        static_obstacles_array = list()
        for i in range(static_obstacles):
            model = random.choice(self._model_loader.models)
            obs = self._create_obstacle(name=model, model=self._model_loader.bind(model))
            static_obstacles_array.append(obs)

        if len(static_obstacles_array):
            self._obstacle_manager.spawn_obstacles(setups=static_obstacles_array)

        # Create interactive obstacles
        interactive_obstacles_array = list()
        for i in range(interactive_obstacles):
            model = random.choice(self._model_loader.models)
            obs = self._create_obstacle(name=model, model=self._model_loader.bind(model))
            interactive_obstacles_array.append(obs)

        if len(interactive_obstacles_array):
            self._obstacle_manager.spawn_obstacles(interactive_obstacles_array)

        # Create dynamic obstacles
        dynamic_obstacles_array = list()
        for i in range(dynamic_obstacles):
            model = random.choice(self._dynamic_model_loader.models)
            obs = self._create_dynamic_obstacle(name=model, model=self._dynamic_model_loader.bind(model))
            dynamic_obstacles_array.append(obs)

        if len(dynamic_obstacles_array):
            self._obstacle_manager.spawn_dynamic_obstacles(dynamic_obstacles_array)

        return False, (0, 0, 0)
