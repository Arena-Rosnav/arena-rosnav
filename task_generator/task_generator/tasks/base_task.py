from abc import abstractmethod
import os
from typing import Any, List

from rospkg import RosPack
import rospy

from rosgraph_msgs.msg import Clock
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from task_generator.constants import Constants
from task_generator.utils import ModelLoader
from task_generator.manager.obstacle_manager import ObstacleManager

class BaseTask():
    """
        Base Task as parent class for all other tasks.
    """

    obstacle_manager: ObstacleManager
    robot_managers: List[Any]
    map_manager: Any

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
