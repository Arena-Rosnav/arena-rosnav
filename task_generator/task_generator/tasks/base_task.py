from abc import abstractmethod
import rospy

from rosgraph_msgs.msg import Clock
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid

from ..manager.obstacle_manager import ObstacleManager
from ..manager.map_manager import MapManager
from ..manager.robot_manager import RobotManager
from task_generator.constants import Constants


class BaseTask:
    """
    Base Task as parent class for all other tasks.
    """

    def __init__(
        self,
        obstacles_manager: ObstacleManager,
        robot_managers: RobotManager,
        map_manager: MapManager,
        *args,
        **kwargs
    ):
        self.obstacles_manager = obstacles_manager
        self.robot_managers = robot_managers
        self.map_manager = map_manager

        rospy.Subscriber("/clock", Clock, self.clock_callback)
        self.last_reset_time = 0
        self.clock = Clock()

        self._set_up_robot_managers()

    def _set_up_robot_managers(self):
        for manager in self.robot_managers:
            manager.set_up_robot()

    def _get_robot_names(self):
        return [manager.namespace for manager in self.robot_managers]

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

        return all(manager.is_done() for manager in self.robot_managers)
