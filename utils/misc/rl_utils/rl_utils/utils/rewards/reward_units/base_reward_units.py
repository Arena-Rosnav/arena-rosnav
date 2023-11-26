from abc import ABC, abstractmethod
from typing import Any

import numpy as np
from scipy.spatial import cKDTree

from ..reward_function import RewardFunction


class RewardUnit(ABC):
    """
    Reward Unit Base Class

    This class represents a reward unit and is an abstract base class (ABC).
    It provides methods for adding rewards and information, checking parameters, and resetting the reward unit.
    It also contains an abstract method for calculating the reward.

    Attributes:
        _reward_function (RewardFunction): The RewardFunction instance holding this unit.
        _on_safe_dist_violation (bool): Whether the unit is applied on safe distance violation.
    """

    def __init__(
        self,
        reward_function: RewardFunction,
        _on_safe_dist_violation: bool = True,
        *args,
        **kwargs
    ) -> None:
        """_summary_

        Args:
            reward_function (RewardFunction): _description_
            _on_safe_dist_violation (bool, optional): _description_. Defaults to True.
        """
        self._reward_function = reward_function
        self._on_safe_dist_violation = _on_safe_dist_violation

    @property
    def on_safe_dist_violation(self):
        return self._on_safe_dist_violation

    def add_reward(self, value: float):
        self._reward_function.add_reward(value=value)

    def add_info(self, info: dict):
        self._reward_function.add_info(info=info)

    def check_parameters(self, *args, **kwargs):
        pass

    def reset(self):
        pass

    @property
    def robot_radius(self):
        return self._reward_function.robot_radius

    @abstractmethod
    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        raise NotImplementedError()


class GlobalplanRewardUnit(RewardUnit, ABC):
    """
    Base Globalplan Reward Unit

    This class represents a globalplan reward unit and is a subclass of RewardUnit.
    It provides methods for calculating the distance to a global plan and resetting the unit.

    Attributes:
        _kdtree: cKDTree
    """

    _kdtree: cKDTree

    def __init__(
        self,
        reward_function: "RewardFunction",
        _on_safe_dist_violation: bool = True,
        *args,
        **kwargs
    ) -> None:
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)
        self._kdtree = None
        self._reward_function.add_global_state_info("curr_dist_to_path", None)

    @property
    def curr_dist_to_path(self) -> float:
        return self._reward_function.get_global_state_info("curr_dist_to_path")

    @curr_dist_to_path.setter
    def curr_dist_to_path(self, value: float) -> None:
        self._reward_function.add_global_state_info("curr_dist_to_path", value)

    def __call__(
        self, global_plan: np.ndarray, robot_pose, *args: Any, **kwargs: Any
    ) -> Any:
        if (
            not self.curr_dist_to_path
            and isinstance(global_plan, np.ndarray)
            and len(global_plan) > 0
        ):
            self.curr_dist_to_path = self.get_dist_to_globalplan(
                global_plan, robot_pose
            )

    def get_dist_to_globalplan(self, global_plan: np.ndarray, robot_pose):
        if self._kdtree is None:
            self._kdtree = cKDTree(global_plan)

        dist, _ = self._kdtree.query([robot_pose.x, robot_pose.y])
        return dist

    def reset(self):
        self._kdtree = None
        self.curr_dist_to_path = None
