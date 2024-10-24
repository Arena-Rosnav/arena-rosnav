"""
ADVANCED OBSERVATIONS: OBSERVATIONS THAT ARE NOT DIRECTLY DERIVED FROM TOPICS
"""

__all__ = [
    "GoalLocationInRobotFrame",
    "DistAngleToGoal",
    "SubgoalLocationInRobotFrame",
    "DistAngleToSubgoal",
    "LaserSafeDistanceGenerator",
]

from typing import TYPE_CHECKING, List, Tuple, Type, TypeVar

import numpy as np
from rl_utils.state_container import SimulationStateContainer

from ..collectors import (
    BaseUnit,
    FullRangeLaserCollector,
    GoalCollector,
    LaserCollector,
    RobotPoseCollector,
    SubgoalCollector,
)
from ..utils.semantic import get_relative_pos_to_robot
from .base_generator import ObservationGeneratorUnit

if TYPE_CHECKING:
    from rl_utils.utils.type_alias.observation import ObservationDict

import rospy


class GoalLocationInRobotFrame(ObservationGeneratorUnit):
    """
    Observation generator unit that calculates the goal location in the robot's frame of reference.
    """

    name: str = "goal_location_in_robot_frame"
    requires: List[BaseUnit] = [GoalCollector, RobotPoseCollector]
    data_class: Type[np.ndarray] = np.ndarray

    def generate(self, obs_dict: "ObservationDict", *args, **kwargs) -> np.ndarray:
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

    def generate(self, obs_dict: "ObservationDict", *args, **kwargs) -> np.ndarray:
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

    def generate(
        self, obs_dict: "ObservationDict", *args, **kwargs
    ) -> Tuple[DistToGoal, AngleToGoal]:
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

    def generate(
        self, obs_dict: "ObservationDict", *args, **kwargs
    ) -> Tuple[DistToGoal, AngleToGoal]:
        goal_in_robot_frame: SubgoalLocationInRobotFrame.data_class = obs_dict[
            SubgoalLocationInRobotFrame.name
        ]

        dist_to_goal = np.linalg.norm(goal_in_robot_frame)
        angle_to_goal = np.arctan2(goal_in_robot_frame[1], goal_in_robot_frame[0])

        return np.array((dist_to_goal, angle_to_goal))


class LaserSafeDistanceGenerator(ObservationGeneratorUnit):
    name: str = "laser_safe_distance_violation"
    requires: List[BaseUnit] = [LaserCollector, FullRangeLaserCollector]
    data_class: Type[bool] = bool

    def generate(
        self,
        obs_dict: "ObservationDict",
        simulation_state_container: SimulationStateContainer,
        *args,
        **kwargs,
    ) -> np.ndarray:
        if not isinstance(simulation_state_container, SimulationStateContainer):
            rospy.logwarn_throttle(
                60,
                f"Can't calculate '{self.name}'-Generator! SimulationStateContainer not provided.",
            )
        return (
            obs_dict[LaserCollector.name].min()
            <= simulation_state_container.robot.safety_distance
            if FullRangeLaserCollector.name not in obs_dict
            else obs_dict[FullRangeLaserCollector.name].min()
            <= simulation_state_container.robot.safety_distance
        )
