import math
from typing import Dict

from builtin_interfaces.msg import Time
from task_generator.shared import PositionOrientation
from task_generator.tasks.robots.random import TM_Random


class TM_Explore(TM_Random):
    """
    This class represents a task manager for exploring robots.
    It inherits from the TM_Random class.
    """

    _timeouts: Dict[str, Time]

    def reset(self, **kwargs):
        super().reset(**kwargs)
        self._timeouts = {}
        for name in self._PROPS.robot_managers.keys():
            self._reset_timeout(name)

    @property
    def done(self) -> bool:
        """
        Checks if the exploration task is done for all robots.

        Returns:
            bool: True if the task is done for all robots, False otherwise.
        """
        for robot, manager in self._PROPS.robot_managers.items():
            if manager.is_done:
                waypoint = self._PROPS.world_manager.get_position_on_map(
                    safe_dist=manager.safe_distance, forbid=False
                )
                self._set_goal(
                    robot,
                    PositionOrientation(
                        waypoint.x, waypoint.y, self.node.conf.General.RNG.value.random() * 2 * math.pi)
                )

            elif (self._PROPS.clock.clock.sec - self._timeouts[robot].sec) > self.node.conf.Robot.TIMEOUT.value:
                waypoint = self._PROPS.world_manager.get_position_on_map(
                    safe_dist=manager.safe_distance, forbid=False
                )
                self._set_position(
                    robot,
                    PositionOrientation(
                        waypoint.x, waypoint.y, self.node.conf.General.RNG.value.random() * 2 * math.pi)
                )

        return False

    def _reset_timeout(self, robot: str):
        """
        Resets the timeout for a specific robot.

        Args:
            robot (str): The name of the robot.
        """
        self._timeouts[robot] = self._PROPS.clock.clock

    def _set_position(self, name: str, position: PositionOrientation):
        """
        Sets the position of a specific robot and resets the timeout.

        Args:
            name (str): The name of the robot.
            position (PositionOrientation): The new position of the robot.
        """
        self._reset_timeout(name)
        self._PROPS.robot_managers[name].reset(position, None)

    def _set_goal(self, name: str, position: PositionOrientation):
        """
        Sets the goal position of a specific robot and resets the timeout.

        Args:
            name (str): The name of the robot.
            position (PositionOrientation): The new goal position of the robot.
        """
        self._reset_timeout(name)
        self._PROPS.robot_managers[name].reset(None, position)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
