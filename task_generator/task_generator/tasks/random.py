import random
import rospy

from task_generator.constants import Constants, TaskMode
from task_generator.tasks.task_factory import TaskFactory

from .base_task import BaseTask


@TaskFactory.register(TaskMode.RANDOM)
class RandomTask(BaseTask):
    """
        The random task spawns static and dynamic
        obstacles on every reset and will create
        a new robot start and goal position for
        each task.
    """

    def reset(
        self, start=None, goal=None,
        static_obstacles=None, dynamic_obstacles=None,
    ):
        return super().reset(
            lambda: self._reset_robot_and_obstacles(
                start=start, goal=goal,
                static_obstacles=static_obstacles,
                dynamic_obstacles=dynamic_obstacles,
            )
        )

    def _reset_robot_and_obstacles(
        self, start=None, goal=None, 
        dynamic_obstacles=None, static_obstacles=None,
    ):
        robot_positions = []

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

        dynamic_obstacles = random.randint(
            TaskMode.Random.MIN_DYNAMIC_OBS,
            TaskMode.Random.MAX_DYNAMIC_OBS
        ) if dynamic_obstacles == None else dynamic_obstacles
        static_obstacles = random.randint(
            TaskMode.Random.MIN_STATIC_OBS,
            TaskMode.Random.MAX_STATIC_OBS
        ) if static_obstacles == None else static_obstacles

        self.obstacles_manager.reset_random(
            dynamic_obstacles=dynamic_obstacles,
            static_obstacles=static_obstacles,
            forbidden_zones=robot_positions
        )

        return False, (0, 0, 0)
