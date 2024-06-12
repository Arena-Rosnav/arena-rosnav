"""
ADVANCED OBSERVATIONS: OBSERVATIONS THAT ARE NOT DIRECTLY DERIVED FROM TOPICS
"""

from typing import Any, Dict, List, Tuple, Type, TypeVar

import numpy as np

from ..collectors import BaseUnit, GoalCollector, RobotPoseCollector, SubgoalCollector
from ..utils.semantic import get_relative_pos_to_robot
from .base_generator import ObservationGeneratorUnit

__all__ = [
    "GoalLocationInRobotFrame",
    "DistAngleToGoal",
    "SubgoalLocationInRobotFrame",
    "DistAngleToSubgoal",
]


ObservationDict = Dict[str, Any]


class GoalLocationInRobotFrame(ObservationGeneratorUnit):
    """
    Observation generator unit that calculates the goal location in the robot's frame of reference.
    """

    name: str = "goal_location_in_robot_frame"
    requires: List[BaseUnit] = [GoalCollector, RobotPoseCollector]
    data_class: Type[np.ndarray] = np.ndarray

    def generate(self, obs_dict: ObservationDict) -> np.ndarray:
        """
        Generates the observation of the goal location in the robot's frame of reference.

        Args:
            obs_dict (ObservationDict): A dictionary containing the required observations.

        Returns:
            np.ndarray: The goal location in the robot's frame of reference.
        """
        goal_pose: GoalCollector.data_class = obs_dict[GoalCollector.name]
        robot_pose: RobotPoseCollector.data_class = obs_dict[RobotPoseCollector.name]

        return get_relative_pos_to_robot(
            robot_pose,
            np.array([[goal_pose["x"], goal_pose["y"], 1]]),
        ).squeeze(0)


class SubgoalLocationInRobotFrame(ObservationGeneratorUnit):
    name: str = "subgoal_location_in_robot_frame"
    requires: List[BaseUnit] = [SubgoalCollector, RobotPoseCollector]
    data_class: Type[np.ndarray] = np.ndarray

    def generate(self, obs_dict: ObservationDict) -> np.ndarray:
        goal_pose: GoalCollector.data_class = obs_dict[SubgoalCollector.name]
        robot_pose: RobotPoseCollector.data_class = obs_dict[RobotPoseCollector.name]

        return get_relative_pos_to_robot(
            robot_pose,
            np.array([[goal_pose["x"], goal_pose["y"], 1]]),
        ).squeeze(0)


DistToGoal = TypeVar("DistToGoal", float, float)
AngleToGoal = TypeVar("AngleToGoal", float, float)


class DistAngleToGoal(ObservationGeneratorUnit):
    name: str = "dist_angle_to_goal"
    requires: List[BaseUnit] = [GoalLocationInRobotFrame]
    data_class: Type[Tuple[DistToGoal, AngleToGoal]] = Tuple[DistToGoal, AngleToGoal]

    def generate(self, obs_dict: ObservationDict) -> Tuple[DistToGoal, AngleToGoal]:
        goal_in_robot_frame: GoalLocationInRobotFrame.data_class = obs_dict[
            GoalLocationInRobotFrame.name
        ]

        dist_to_goal = np.linalg.norm(goal_in_robot_frame)
        angle_to_goal = np.arctan2(goal_in_robot_frame[1], goal_in_robot_frame[0])

        return np.array((dist_to_goal, angle_to_goal))


class DistAngleToSubgoal(ObservationGeneratorUnit):
    name: str = "dist_angle_to_subgoal"
    requires: List[BaseUnit] = [SubgoalLocationInRobotFrame]
    data_class: Type[Tuple[DistToGoal, AngleToGoal]] = Tuple[DistToGoal, AngleToGoal]

    def generate(self, obs_dict: ObservationDict) -> Tuple[DistToGoal, AngleToGoal]:
        goal_in_robot_frame: SubgoalLocationInRobotFrame.data_class = obs_dict[
            SubgoalLocationInRobotFrame.name
        ]

        dist_to_goal = np.linalg.norm(goal_in_robot_frame)
        angle_to_goal = np.arctan2(goal_in_robot_frame[1], goal_in_robot_frame[0])

        return np.array((dist_to_goal, angle_to_goal))
