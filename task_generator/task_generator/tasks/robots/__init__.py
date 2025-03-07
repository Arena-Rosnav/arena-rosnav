from task_generator.shared import PositionOrientation
from task_generator.tasks import TaskMode


class TM_Robots(TaskMode):
    """
    Task mode for controlling one or multiple robots.

    Args:
        **kwargs: Additional keyword arguments.

    Attributes:
        _PROPS (TaskProperties): Task properties object.

    """

    _last_reset: int

    def __init__(self, **kwargs):
        TaskMode.__init__(self, **kwargs)

    def reset(self, **kwargs):
        self._last_reset = self._PROPS.clock.clock.sec

    def set_position(self, position: PositionOrientation):
        """
        Set the position of all robots.

        Args:
            position (PositionOrientation): The desired position and orientation.

        """
        for robot_manager in self._PROPS.robot_managers.values():
            robot_manager.reset(position, None)

    def set_goal(self, position: PositionOrientation):
        """
        Set the goal position for all robots.

        Args:
            position (PositionOrientation): The desired goal position and orientation.

        """
        for robot_manager in self._PROPS.robot_managers.values():
            robot_manager.reset(None, position)

    @property
    def done(self):
        """
        Check if all robots have completed their tasks.

        Returns:
            bool: True if all robots are done, False otherwise.

        """
        if (self._PROPS.clock.clock.sec - self._last_reset) \
                > self.node.conf.Robot.TIMEOUT.value:
            return True

        return len(self._PROPS.robot_managers) and all(
            robot_manager.is_done for robot_manager in self._PROPS.robot_managers.values())
