from typing import List
from task_generator.constants import Constants, TaskConfig
from task_generator.shared import Obstacle, PositionOrientation
from task_generator.tasks import Props_, TaskMode


class TM_Robots(TaskMode):
    """
    Task mode for controlling one or multiple robots.

    Args:
        **kwargs: Additional keyword arguments.

    Attributes:
        _PROPS (TaskProperties): Task properties object.

    """

    def __init__(self, **kwargs):
        TaskMode.__init__(self, **kwargs)

    def reset(self, **kwargs):
        ...

    def set_position(self, position: PositionOrientation):
        """
        Set the position of all robots.

        Args:
            position (PositionOrientation): The desired position and orientation.

        """
        for robot in self._PROPS.robot_managers:
            robot.reset(position, None)

    def set_goal(self, position: PositionOrientation):
        """
        Set the goal position for all robots.

        Args:
            position (PositionOrientation): The desired goal position and orientation.

        """
        for robot in self._PROPS.robot_managers:
            robot.reset(None, position)

    @property
    def done(self):
        """
        Check if all robots have completed their tasks.

        Returns:
            bool: True if all robots are done, False otherwise.

        """
        return all(robot.is_done for robot in self._PROPS.robot_managers)
