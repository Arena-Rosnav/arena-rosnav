import random
import rospy
import time

from task_generator.constants import Constants, TaskMode
from task_generator.tasks.task_factory import TaskFactory

from task_generator.shared import DynamicObstacle, Obstacle, DynamicObstacleConfig, ForbiddenZone, Model, ModelType, ObstacleConfig, Waypoint

from .base_task import BaseTask

import xml.etree.ElementTree as ET

dynamic_obstacles_random = random.randint(TaskMode.Random.MIN_DYNAMIC_OBS,TaskMode.Random.MAX_DYNAMIC_OBS)
static_obstacles_random = random.randint(TaskMode.Random.MIN_STATIC_OBS,TaskMode.Random.MAX_STATIC_OBS)
interactive_obstacles_random = random.randint(TaskMode.Random.MIN_INTERACTIVE_OBS,TaskMode.Random.MAX_INTERACTIVE_OBS)

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

        self.obstacle_manager.reset_random(
            dynamic_obstacles=dynamic_obstacles_random,
            static_obstacles=static_obstacles_random,
            interactive_obstacles=interactive_obstacles_random,
            forbidden_zones=robot_positions
        )

        return False, (0, 0, 0)


    #moved from obstacle_manager
    def reset_random(
            self, 
            dynamic_obstacles=Constants.ObstacleManager.DYNAMIC_OBSTACLES,
            static_obstacles=Constants.ObstacleManager.STATIC_OBSTACLES,
            interactive_obstacles=Constants.ObstacleManager.INTERACTIVE_OBSTACLES,
            forbidden_zones=None
        ):

        if forbidden_zones is None:
            forbidden_zones = []

        self.forbidden_zones = forbidden_zones

        if self.first_reset:
            self.first_reset = False
        else:  
            self.dynamic_manager.remove_obstacles()

        dynamic_obstacles_array = np.array([],dtype=object).reshape(0,3)
        static_obstacles_array = np.array([],dtype=object).reshape(0,2)
        interactive_obstacles_array = np.array([],dtype=object).reshape(0,2)

        forbidden_zones = forbidden_zones + self.spawn_map_obstacles()

        # Create static obstacles
        for i in range(static_obstacles):
            x = self.create_obstacle(False, forbidden_zones)
            forbidden_zones.append([x.position[0], x.position[1], 40])
            static_obstacles_array = np.vstack((static_obstacles_array, x))

        if static_obstacles_array.size > 0:
            self.dynamic_manager.spawn_obstacles(static_obstacles_array, interaction_radius=0.0)

        # Create interactive obstacles  
        for i in range(interactive_obstacles):
            x = self.create_obstacle(False, forbidden_zones)
            forbidden_zones.append([x.position[0], x.position[1], 40])
            interactive_obstacles_array = np.vstack((interactive_obstacles_array, x))

        if interactive_obstacles_array.size > 0:
            self.dynamic_manager.spawn_obstacles(interactive_obstacles_array, interaction_radius=1.0)

        # Create dynamic obstacles 
        for i in range(dynamic_obstacles):
            x = self.create_obstacle(True, forbidden_zones)
            dynamic_obstacles_array = np.vstack((dynamic_obstacles_array, x))

        if dynamic_obstacles_array.size > 0:
            self.dynamic_manager.spawn_dynamic_obstacle(dynamic_obstacles_array)