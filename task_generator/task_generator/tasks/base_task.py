from dataclasses import asdict
import os
from typing import Callable, Dict, List, Optional
import numpy as np

from rospkg import RosPack
import rospy

from rosgraph_msgs.msg import Clock
from task_generator.constants import Constants
from task_generator.manager.map_manager import MapManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.utils import ModelLoader
from task_generator.manager.obstacle_manager import ObstacleManager

from task_generator.shared import DynamicObstacle, Model, ModelWrapper, Obstacle, PositionOrientation, Waypoint


class BaseTask():
    """
        Base Task as parent class for all other tasks.
    """

    _obstacle_manager: ObstacleManager
    _robot_managers: List[RobotManager]
    _map_manager: MapManager

    _model_loader: ModelLoader
    _dynamic_model_loader: ModelLoader

    _clock: Clock
    _last_reset_time: int

    def __init__(self, obstacle_manager: ObstacleManager, robot_managers: List[RobotManager], map_manager: MapManager, *args, **kwargs):
        self._obstacle_manager = obstacle_manager
        self._robot_managers = robot_managers
        self._map_manager = map_manager

        rospy.Subscriber("/clock", Clock, self._clock_callback)
        self._last_reset_time = 0
        self._clock = Clock()

        self._set_up_robot_managers()

        self._model_loader = ModelLoader(os.path.join(
            RosPack().get_path("arena-simulation-setup"), "obstacles"))
        self._dynamic_model_loader = ModelLoader(os.path.join(
            RosPack().get_path("arena-simulation-setup"), "dynamic_obstacles"))

    def reset(self, callback: Optional[Callable] = None) -> bool:
        """
            Calls a passed reset function (usually the tasks own reset)
            inside a loop so when the callback fails once it is tried
            again. After MAX_RESET_FAIL_TIMES the reset is considered
            as fail and the simulation is shut down.
        """
        fails = 0
        return_val = False

        self._last_reset_time = self._clock.clock.secs

        while fails < Constants.MAX_RESET_FAIL_TIMES:
            try:
                return_val = return_val if callback is None else callback()
                break

            except rospy.ServiceException as e:
                rospy.logwarn(repr(e))
                fails += 1

        else:
            rospy.signal_shutdown("Reset error!")
            raise Exception("reset error!")

        return return_val

    @property
    def is_done(self) -> bool:
        if self._clock.clock.secs - self._last_reset_time > Constants.TIMEOUT:
            return True

        for manager in self._robot_managers:
            if not manager.is_done:
                return False

        return True

    @property
    def robot_names(self) -> List[str]:
        return [manager.name for manager in self._robot_managers]

    def _set_up_robot_managers(self):
        for manager in self._robot_managers:
            manager.set_up_robot()

    def _clock_callback(self, clock: Clock):
        self._clock = clock


class CreateObstacleTask(BaseTask):
    """
        Extends BaseTask with a create_obstacle method that provides a convenient way to create random obstacles.
    """
    # moved from obstacle manager


    def _create_dynamic_obstacle(self, name:str, model: ModelWrapper, waypoints: Optional[List[Waypoint]] = None, extra: Optional[Dict] = None, **kwargs) -> DynamicObstacle:

        setup = self._create_obstacle(name=name, model=model, extra=extra, **kwargs)

        if waypoints is None:

            safe_distance = 0.5
            waypoints = [(*setup.position[:2], safe_distance)]  # the first waypoint
            safe_distance = 0.1  # the other waypoints don't need to avoid robot
            for j in range(10):
                dist = 0
                while dist < 8:
                    [x2, y2, *
                        _] = self._map_manager.get_random_pos_on_map(safe_distance)
                    dist = np.linalg.norm(
                        [waypoints[-1][0] - x2, waypoints[-1][1] - y2])
                    waypoints.append((x2, y2, 1))

        return DynamicObstacle(**{
            **asdict(setup),
            **dict(waypoints=waypoints)
        })

    def _create_obstacle(self, name:str, model: ModelWrapper, position: Optional[PositionOrientation] = None, extra: Optional[Dict] = None, **kwargs) -> Obstacle:
        """ 
        Creates and returns a newly generated obstacle of requested type: 
        """

        safe_distance = 0.5
        
        if position is None:
            point: Waypoint = self._map_manager.get_random_pos_on_map(safe_distance)
            position = (point[0], point[1], np.pi * np.random.random())

        if extra is None:
            extra = dict()

        return Obstacle(
            position=position,
            name=name,
            model=model,
            extra=extra,
            **kwargs
        )