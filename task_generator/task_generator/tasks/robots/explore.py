import math
from typing import Dict

import genpy
from task_generator.constants import Config, Constants
from task_generator.shared import PositionOrientation
from task_generator.tasks.robots.random import TM_Random
from task_generator.tasks.task_factory import TaskFactory

@TaskFactory.register_robots(Constants.TaskMode.TM_Robots.EXPLORE)
class TM_Explore(TM_Random):
    """
    This class represents a task manager for exploring robots.
    It inherits from the TM_Random class.
    """

    _timeouts: Dict[int, genpy.Time]

    @classmethod
    def prefix(cls, *args):
        return super().prefix("explore", *args)
        

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self._timeouts = dict()
        for i in range(len(self._PROPS.robot_managers)):
            self._reset_timeout(i)

    @property
    def done(self) -> bool:
        """
        Checks if the exploration task is done for all robots.

        Returns:
            bool: True if the task is done for all robots, False otherwise.
        """
        for i, robot in enumerate(self._PROPS.robot_managers):
            if robot.is_done:
                waypoint = self._PROPS.world_manager.get_position_on_map(
                    safe_dist=robot._robot_radius, forbid=False
                )
                self._set_goal(
                    i, PositionOrientation(*waypoint, Config.General.RNG.random() * 2 * math.pi)
                )

            elif (self._PROPS.clock.clock - self._timeouts[i]).secs > Config.Robot.TIMEOUT:
                waypoint = self._PROPS.world_manager.get_position_on_map(
                    safe_dist=robot._robot_radius, forbid=False
                )
                self._set_position(
                    i, PositionOrientation(*waypoint, Config.General.RNG.random() * 2 * math.pi)
                )

        return False

    def _reset_timeout(self, index: int):
        """
        Resets the timeout for a specific robot.

        Args:
            index (int): The index of the robot.
        """
        self._timeouts[index] = self._PROPS.clock.clock

    def _set_position(self, index: int, position: PositionOrientation):
        """
        Sets the position of a specific robot and resets the timeout.

        Args:
            index (int): The index of the robot.
            position (PositionOrientation): The new position of the robot.
        """
        self._reset_timeout(index)
        self._PROPS.robot_managers[index].reset(position, None)

    def _set_goal(self, index: int, position: PositionOrientation):
        """
        Sets the goal position of a specific robot and resets the timeout.

        Args:
            index (int): The index of the robot.
            position (PositionOrientation): The new goal position of the robot.
        """
        self._reset_timeout(index)
        self._PROPS.robot_managers[index].reset(None, position)
