from abc import abstractmethod
import os
from typing import Any, List, overload
import numpy as np

from rospkg import RosPack
import rospy

from rosgraph_msgs.msg import Clock
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from task_generator.constants import Constants
from task_generator.manager.map_manager import MapManager
from task_generator.utils import ModelLoader
from task_generator.manager.obstacle_manager import ObstacleManager

from task_generator.shared import DynamicObstacle, Obstacle, DynamicObstacleConfig, ForbiddenZone, Model, ModelType, ObstacleConfig, Waypoint
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion


class BaseTask():
    """
        Base Task as parent class for all other tasks.
    """

    obstacle_manager: ObstacleManager
    robot_managers: List[Any]
    map_manager: MapManager

    model_loader: ModelLoader
    dynamic_model_loader: ModelLoader

    clock: Clock
    last_reset_time: int

    def __init__(self, obstacle_manager: ObstacleManager, robot_managers, map_manager, *args, **kwargs):
        self.obstacle_manager = obstacle_manager
        self.robot_managers = robot_managers
        self.map_manager = map_manager

        rospy.Subscriber("/clock", Clock, self.clock_callback)
        self.last_reset_time = 0
        self.clock = Clock()

        self._set_up_robot_managers()

        self.model_loader = ModelLoader(os.path.join(RosPack().get_path("arena-simulation-setup"), "obstacles"))
        self.dynamic_model_loader = ModelLoader(os.path.join(RosPack().get_path("arena-simulation-setup"), "dynamic_obstacles"))

    def _set_up_robot_managers(self):
        for manager in self.robot_managers:
            manager.set_up_robot()

    def _get_robot_names(self):
        names = []

        for manager in self.robot_managers:
            names.append(manager.namespace)

        return names

    def set_robot_names_param(self):
        names = self._get_robot_names()

        rospy.set_param("/robot_names", names)

        
    def reset(self, callback):
        """
            Calls a passed reset function (usually the tasks own reset)
            inside a loop so when the callback fails once it is tried
            again. After MAX_RESET_FAIL_TIMES the reset is considered
            as fail and the simulation is shut down.
        """
        fails = 0
        return_val = False, None 

        self.last_reset_time = self.clock.clock.secs

        while fails < Constants.MAX_RESET_FAIL_TIMES:
            try:
                return_val = callback()

                break
            except rospy.ServiceException as e:
                rospy.logwarn(repr(e))
                fails += 1

        if fails >= Constants.MAX_RESET_FAIL_TIMES:
            rospy.signal_shutdown("Reset error!")
            raise Exception("reset error!")

        return return_val

    def clock_callback(self, clock):
        self.clock = clock

    def is_done(self):
        if self.clock.clock.secs - self.last_reset_time > Constants.TIMEOUT:
            return True
        
        for manager in self.robot_managers:
            if not manager.is_done():
                return False
        
        return True





class CreateObstacleTask(BaseTask):
    """
        Extends BaseTask with a create_obstacle method that provides a convenient way to create random obstacles.
    """
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