import os
import random
from typing import List, Optional, overload
import numpy as np

from rospkg import RosPack
import rospy
import time

from task_generator.constants import Constants, TaskMode
from task_generator.tasks.task_factory import TaskFactory

from .base_task import BaseTask

from task_generator.shared import DynamicObstacle, Obstacle, DynamicObstacleConfig, ForbiddenZone, Model, ModelType, ObstacleConfig, Waypoint
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

import xml.etree.ElementTree as ET

@TaskFactory.register(TaskMode.RANDOM_SCENARIO)
class RandomScenarioTask(BaseTask):
    """
        The random task spawns static and dynamic
        obstacles on every reset and will create
        a new robot start and goal position for
        each task.
    """

    forbidden_zones: List[ForbiddenZone]

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
        self.forbidden_zones = []

        self.obstacle_manager.reset()

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

        xml_path = os.path.join(
        RosPack().get_path("task_generator"), 
        "scenarios", 
        "random_scenario.xml")
            
        tree = ET.parse(xml_path)
        root = tree.getroot()
        num_tables  = [int(str(root[0][0].text)), root[0][1].text, root[0][2].text]
        num_shelves = [int(str(root[1][0].text)), root[1][1].text, root[1][2].text]
        num_adults  = [int(str(root[2][0].text)), root[2][1].text, root[2][2].text]
        num_elder   = [int(str(root[3][0].text)), root[3][1].text, root[3][2].text]
        num_child   = [int(str(root[4][0].text)), root[4][1].text, root[4][2].text]

        dynamic_obstacles_array: List[DynamicObstacle]
        static_obstacles_array: List[Obstacle]
        interactive_obstacles_array: List[Obstacle]

        print(1)
        self.obstacle_manager.spawn_map_obstacles()

        # Create static obstacles
        for ob_type in [num_tables]:

            with open(os.path.join(RosPack().get_path('pedsim_gazebo_plugin'), "models", f"{ob_type[1]}.sdf")) as f:
                model = Model(type=ModelType.SDF, description=f.read(), name=ob_type[1])

            static_obstacles_array = list()
            for i in range(ob_type[0]):
                obstacle = self.create_obstacle(ObstacleConfig(model=model))
                self.forbidden_zones.append((obstacle.pose.position.x, obstacle.pose.position.y, 40))
                static_obstacles_array.append(obstacle)

            if len(static_obstacles_array):
                self.obstacle_manager.spawn_obstacles(obstacles=static_obstacles_array)

        # Create interactive obstacles  
        print(2)
        for ob_type in [num_shelves]:

            with open(os.path.join(RosPack().get_path('pedsim_gazebo_plugin'), "models", f"{ob_type[1]}.sdf")) as f:
                 model = Model(type=ModelType.SDF, description=f.read(), name=ob_type[1])

            interactive_obstacles_array = list()
            for i in range(ob_type[0]):
                obstacle = self.create_obstacle(ObstacleConfig(model=model))
                self.forbidden_zones.append((obstacle.pose.position.x, obstacle.pose.position.y, 40))
                interactive_obstacles_array.append(obstacle)

            if len(interactive_obstacles_array):
                self.obstacle_manager.spawn_obstacles(obstacles=interactive_obstacles_array)

        # Create dynamic obstacles 
        print(3)
        for ob_type in [num_adults,num_elder,num_child]:
            dynamic_obstacles_array = list()
            for i in range(ob_type[0]):
                obstacle = self.create_obstacle(DynamicObstacleConfig(model=self.obstacle_manager.dynamic_manager.default_actor_model))
                dynamic_obstacles_array.append(obstacle)

            if len(dynamic_obstacles_array):
                print("4ter")
                self.obstacle_manager.spawn_dynamic_obstacles(obstacles=dynamic_obstacles_array)

            print(5, ob_type)

        print(6)

        return False, (0, 0, 0)




# moved from obstacle manager

    @overload
    def create_obstacle(self, config: DynamicObstacleConfig) -> DynamicObstacle:
        ...
    @overload
    def create_obstacle(self, config: ObstacleConfig) -> Obstacle:
        ...
    def create_obstacle(self, config: ObstacleConfig) -> Obstacle:
        """ 
        Creates and returns a newly generated obstacle of requested type: 
        """

        safe_distance = 0.5

        if isinstance(config, DynamicObstacleConfig):

            if config.position is None:
                point: Waypoint = self.map_manager.get_random_pos_on_map(safe_distance)
                config.position = (point[0], point[1], 0)

            print("gate")

            if config.waypoints is None:
                config.waypoints = [config.position] # the first waypoint
                safe_distance = 0.1 # the other waypoints don't need to avoid robot
                for j in range(10): 
                    dist = 0
                    while dist < 8:
                        [x2, y2, *_] = self.map_manager.get_random_pos_on_map(safe_distance)
                        dist = np.linalg.norm([config.waypoints[-1][0] - x2, config.waypoints[-1][1] - y2])
                        config.waypoints.append((x2, y2, 1))
            
            return DynamicObstacle(
                name=config.model.name,
                model=config.model,
                pose=Pose(
                    position=Point(*config.position),
                    orientation=Quaternion(x=0, y=0, z=0, w=1)
                ),
                waypoints=config.waypoints
            )
        
        elif isinstance(config, ObstacleConfig):

            if config.position is None:
                point: Waypoint = self.map_manager.get_random_pos_on_map(safe_distance)
                config.position = (point[0], point[1], 0)

            safe_distance = 0.5

            return Obstacle(
                name=config.model.name,
                model=config.model,
                pose=Pose(
                    position=Point(*config.position),
                    orientation=Quaternion(x=0, y=0, z=0, w=1)
                )
            )
        
        else:
            raise ValueError()
        