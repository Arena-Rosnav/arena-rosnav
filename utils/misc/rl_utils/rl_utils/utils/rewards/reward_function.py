from tools.general import load_rew_fnc

import rospy
import numpy as np

from .constants import REWARD_CONSTANTS


class RewardFunction:
    def __init__(self, holonomic, robot_radius, goal_radius, safe_dist):
        self._holonomic = holonomic
        self._robot_radius = robot_radius
        self._safe_dist = safe_dist
        self._goal_radius = goal_radius

        self._last_goal_dist: float = None
        self._last_dist_to_path: float = None
        self._last_action: np.ndarray = None
        self._curr_dist_to_path: float = None
        self._safe_dist_breached: bool = None
        self._kdtree: np.ndarray = None

        self._curr_reward = 0
        self._info = {}

    @property
    def robot_radius(self) -> float:
        return self.robot_radius

    @property
    def goal_radius(self) -> float:
        return self._goal_radius

    @goal_radius.setter
    def goal_radius(self, value) -> None:
        if value < REWARD_CONSTANTS.MIN_GOAL_RADIUS:
            raise ValueError(
                f"Goal radius smaller than {REWARD_CONSTANTS.MIN_GOAL_RADIUS}"
            )
        self._goal_radius = value

    @property
    def curr_dist_to_path(self) -> float:
        return self._curr_dist_to_path

    @property
    def safe_dist_breached(self) -> bool:
        return self._safe_dist_breached

    def set_safe_dist_breached(self, laser_scan: np.ndarray) -> None:
        self._safe_dist_breached = laser_scan.min() <= self._safe_dist

    def add_reward(self, value: float):
        self._curr_reward += value

    def add_info(self, info: dict):
        self._info.update(info)

    def _reset(self):
        self._curr_reward = 0
        self._info = {}

    def reset(self):
        self.goal_radius = rospy.get_param("/goal_radius", 0.3)

        self._last_goal_dist = None
        self._last_dist_to_path = None
        self._last_action = None
        self._kdtree = None
        self._curr_dist_to_path = None

    def calculate_reward(self, *args, **kwargs):
        self.set_safe_dist_breached(kwargs["laser_scan"])
