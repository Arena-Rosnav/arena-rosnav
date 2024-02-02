from typing import Dict, List
import rospy
from task_generator.constants import Constants
from task_generator.shared import PositionOrientation
from task_generator.tasks.robots.random import TM_Random
from task_generator.tasks.task_factory import TaskFactory


@TaskFactory.register_robots(Constants.TaskMode.TM_Robots.GUIDED)
class TM_Guided(TM_Random):
    """
    A class representing a guided task manager for robots.

    Inherits from TM_Random.

    Attributes:
        PARAM_WAYPOINTS (str): The parameter name for storing the guided waypoints.
        _waypoints (List[PositionOrientation]): The list of waypoints for the guided task.
        _waypoint_states (Dict[str, int]): The dictionary storing the current waypoint state for each robot.

    Methods:
        __init__(self, **kwargs): Initializes the TM_Guided object.
        reset(self, **kwargs): Resets the TM_Guided object.
        done(self): Checks if the guided task is done.
        set_position(self, position: PositionOrientation): Sets the position for the guided task.
        set_goal(self, position: PositionOrientation): Sets the goal for the guided task.
        _reset_waypoints(self, *args, **kwargs): Resets the waypoints for the guided task.
    """

    PARAM_WAYPOINTS = "guided_waypoints"

    _waypoints: List[PositionOrientation]
    _waypoint_states: Dict[str, int]

    @classmethod
    def prefix(cls, *args):
        return super().prefix("guided", *args)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self._waypoints = []
        self._waypoint_states = {robot.name: 0 for robot in self._PROPS.robot_managers}
        self._reset_waypoints()

    def reset(self, **kwargs):
        """
        Resets the TM_Guided object.

        Args:
            **kwargs: Additional keyword arguments.

        Returns:
            None
        """
        super().reset(**kwargs)
        self._reset_waypoints()

    @property
    def done(self):
        """
        Checks if the guided task is done.

        Returns:
            bool: True if the guided task is done, False otherwise.
        """
        for robot in self._PROPS.robot_managers:
            if robot.is_done:
                waypoints = self._waypoints or [None]
                self._waypoint_states[robot.name] += 1
                self._waypoint_states[robot.name] %= len(waypoints)
                robot.reset(
                    start_pos=None,
                    goal_pos=waypoints[self._waypoint_states[robot.name]],
                )

        return False

    def set_position(self, position: PositionOrientation):
        """
        Sets the position for the guided task.

        Args:
            position (PositionOrientation): The position to set.

        Returns:
            None
        """
        self._reset_waypoints()

    def set_goal(self, position: PositionOrientation):
        """
        Sets the goal for the guided task.

        Args:
            position (PositionOrientation): The goal position to set.

        Returns:
            None
        """
        self._waypoints.append(position)
        rospy.set_param(self.PARAM_WAYPOINTS, [tuple(wp) for wp in self._waypoints])

        if len(self._waypoints) == 1:
            for robot in self._PROPS.robot_managers:
                robot.reset(None, position)

    def _reset_waypoints(self, *args, **kwargs):
        """
        Resets the waypoints for the guided task.

        Args:
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.

        Returns:
            None
        """
        for robot in self._waypoint_states:
            self._waypoint_states[robot] = 0

        for robot in self._PROPS.robot_managers:
            robot.reset(robot.start_pos, robot.start_pos)

        self._waypoints = []
        rospy.set_param(self.PARAM_WAYPOINTS, self._waypoints)
