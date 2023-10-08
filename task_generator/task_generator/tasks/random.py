import random
from typing import List


from task_generator.constants import Constants, TaskMode
from task_generator.tasks.task_factory import TaskFactory

from task_generator.shared import DynamicObstacle, Obstacle, DynamicObstacleConfig, ObstacleConfig

from task_generator.tasks.base_task import CreateObstacleTask


dynamic_obstacles_random = random.randint(
    TaskMode.Random.MIN_DYNAMIC_OBS, TaskMode.Random.MAX_DYNAMIC_OBS)
static_obstacles_random = random.randint(
    TaskMode.Random.MIN_STATIC_OBS, TaskMode.Random.MAX_STATIC_OBS)
interactive_obstacles_random = random.randint(
    TaskMode.Random.MIN_INTERACTIVE_OBS, TaskMode.Random.MAX_INTERACTIVE_OBS)


@TaskFactory.register(TaskMode.RANDOM)
class RandomTask(CreateObstacleTask):
    """
        The random task spawns static and dynamic
        obstacles on every reset and will create
        a new robot start and goal position for
        each task.
    """

    def reset(
        self, start=None, goal=None,
        static_obstacles: int = 0, dynamic_obstacles: int = 0,
    ):
        return super().reset(
            lambda: self._reset_robot_and_obstacles(
                start=start, goal=goal,
                static_obstacles=static_obstacles,
                dynamic_obstacles=dynamic_obstacles,
            )
        )

    def _reset_robot_and_obstacles(
        self,
        start=None,
        goal=None,
        dynamic_obstacles: int = 0,
        static_obstacles: int = 0
    ):
        robot_positions = []

        interactive_obstacles: int = 0

        for manager in self.robot_managers:
            for pos in manager.reset(
                forbidden_zones=robot_positions
            ):
                robot_positions.append(
                    [
                        pos[0],
                        pos[1],
                        manager.robot_radius + Constants.RobotManager.SPAWN_ROBOT_SAFE_DIST
                    ]
                )

        self.obstacle_manager.reset()

        dynamic_obstacles_array: List[DynamicObstacle]
        static_obstacles_array: List[Obstacle]
        interactive_obstacles_array: List[Obstacle]

        self.obstacle_manager.spawn_map_obstacles()

        # Create static obstacles
        static_obstacles_array = list()
        for i in range(static_obstacles):
            model = self.model_loader.load(
                random.choice(self.model_loader.models))
            obs = self.create_obstacle(ObstacleConfig(model=model))
            static_obstacles_array.append(obs)

        if len(static_obstacles_array):
            self.obstacle_manager.spawn_obstacles(
                obstacles=static_obstacles_array)

        # Create interactive obstacles
        interactive_obstacles_array = list()
        for i in range(interactive_obstacles):
            model = self.model_loader.load(
                random.choice(self.model_loader.models))
            obs = self.create_obstacle(ObstacleConfig(model=model))
            interactive_obstacles_array.append(obs)

        if len(interactive_obstacles_array):
            self.obstacle_manager.spawn_obstacles(interactive_obstacles_array)

        # Create dynamic obstacles
        dynamic_obstacles_array = list()
        for i in range(dynamic_obstacles):
            model = self.dynamic_model_loader.load(
                random.choice(self.dynamic_model_loader.models))
            obs = self.create_obstacle(DynamicObstacleConfig(model=model))
            dynamic_obstacles_array.append(obs)

        if len(dynamic_obstacles_array):
            self.obstacle_manager.spawn_dynamic_obstacles(
                dynamic_obstacles_array)

        return False, (0, 0, 0)
