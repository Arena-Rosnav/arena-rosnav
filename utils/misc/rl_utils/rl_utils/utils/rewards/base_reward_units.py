from abc import ABC, abstractmethod
from typing import Any

import numpy as np
from scipy.spatial import cKDTree

from .reward_function import RewardFunction


class RewardUnit(ABC):
    def __init__(
        self,
        reward_function: RewardFunction,
        _on_safe_dist_violation: bool = True,
        *args,
        **kwargs
    ) -> None:
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
    def __init__(
        self,
        reward_function: "RewardFunction",
        _on_safe_dist_violation: bool = True,
        *args,
        **kwargs
    ) -> None:
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)
        self._kdtree = None

    @property
    def curr_dist_to_path(self) -> float:
        return self._reward_function.curr_dist_to_path

    @curr_dist_to_path.setter
    def curr_dist_to_path(self, value: float) -> None:
        self._reward_function.curr_dist_to_path = value

    @property
    def safe_dist_breached(self) -> bool:
        return self._reward_function.safe_dist_breached

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
