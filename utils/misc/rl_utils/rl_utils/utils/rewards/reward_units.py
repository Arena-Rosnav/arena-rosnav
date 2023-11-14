from typing import Any

from scipy.spatial import cKDTree
from tools.general import load_rew_fnc

from abc import ABC, abstractmethod
import numpy as np
from warnings import warn

from .constants import REWARD_CONSTANTS
from .reward_function import RewardFunction


class RewardUnit(ABC):
    def __init__(self, reward_function: RewardFunction, *args, **kwargs) -> None:
        self._reward_function = reward_function
        self.check_parameters(*args, **kwargs)

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


class RewardUnitGlobalplan(RewardUnit, ABC):
    def __init__(self, reward_function: RewardFunction, *args, **kwargs) -> None:
        super().__init__(reward_function, *args, **kwargs)
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

    def get_dist_to_globalplan(self, global_plan: np.ndarray, robot_pose):
        if self._kdtree is None:
            self._kdtree = cKDTree(global_plan)

        dist, _ = self._kdtree.query([robot_pose.x, robot_pose.y])
        return dist

    def reset(self):
        self._kdtree = None


class RewardGoalReached(RewardUnit):
    def __init__(
        self, reward_function: RewardFunction, reward: float = 15.0, *args, **kwargs
    ):
        self._reward = reward
        self._goal_radius = self._reward_function.goal_radius
        super().__init__(reward_function, *args, **kwargs)

    def check_parameters(self, *args, **kwargs):
        if self._reward < 0.0:
            warn_msg = (
                f"[{self.__class__.__name__}] Reconsider this reward. "
                f"Negative rewards may lead to unfavorable behaviors. "
                f"Current value: {self._reward}"
            )
            warn(warn_msg)

    def __call__(self, distance_to_goal: float, *args: Any, **kwargs: Any) -> Any:
        if distance_to_goal < self._reward_function.goal_radius:
            self.add_reward(self._reward)
            self.add_info({"is_done": True, "done_reason": 2, "is_success": True})
        else:
            self.add_info({"is_done": False})

    def reset(self):
        self._goal_radius = self._reward_function.goal_radius


class RewardSafeDistance(RewardUnit):
    def __init__(
        self, reward_function: RewardFunction, reward: float = 0.15, *args, **kwargs
    ):
        self._reward = reward
        self._safe_dist = self._reward_function._safe_dist
        super().__init__(reward_function, *args, **kwargs)

    def check_parameters(self, *args, **kwargs):
        if self._reward > 0.0:
            warn_msg = (
                f"[{self.__class__.__name__}] Reconsider this reward. "
                f"Positive rewards may lead to unfavorable behaviors. "
                f"Current value: {self._reward}"
            )
            warn(warn_msg)

    def __call__(self, laser_scan: np.ndarray, *args: Any, **kwargs: Any):
        violation_in_blind_spot = False
        if "full_laser_scan" in kwargs:
            violation_in_blind_spot = kwargs["full_laser_scan"].min() <= self._safe_dist

        if laser_scan.min() < self._safe_dist or violation_in_blind_spot:
            self.add_reward(self._reward)
            self.add_info({"safe_dist_violation": True})


class RewardNoMovement(RewardUnit):
    def __init__(
        self, reward_function: RewardFunction, reward: float = -0.01, *args, **kwargs
    ):
        self._reward = reward
        super().__init__(reward_function, *args, **kwargs)

    def check_parameters(self, *args, **kwargs):
        if self._reward > 0.0:
            warn_msg = (
                f"[{self.__class__.__name__}] Reconsider this reward. "
                f"Positive rewards may lead to unfavorable behaviors. "
                f"Current value: {self._reward}"
            )
            warn(warn_msg)

    def __call__(self, action: np.ndarray, *args: Any, **kwargs: Any):
        if (
            action is not None
            and abs(action[0]) <= REWARD_CONSTANTS.NO_MOVEMENT_TOLERANCE
        ):
            self.add_reward(self._reward)


class RewardGoalApproach(RewardUnit):
    def __init__(
        self,
        reward_function: RewardFunction,
        pos_factor: float = 0.3,
        neg_factor: float = 0.5,
        *args,
        **kwargs,
    ):
        self._pos_factor = pos_factor
        self._neg_factor = neg_factor

        super().__init__(reward_function, *args, **kwargs)

        self.last_goal_dist = None

    def check_parameters(self, *args, **kwargs):
        if self._pos_factor < 0 or self._neg_factor:
            warn_msg = (
                f"[{self.__class__.__name__}] Both factors should be positive. "
                f"Current values: [pos_factor={self._pos_factor}], [neg_factor={self._neg_factor}]"
            )
            warn(warn_msg)
        if self._pos_factor >= self._neg_factor:
            warn_msg = (
                "'pos_factor' should be smaller than 'neg_factor' otherwise rotary trajectories will get rewarded. "
                f"Current values: [pos_factor={self._pos_factor}], [neg_factor={self._neg_factor}]"
            )
            warn(warn_msg)

    def __call__(self, distance_to_goal, *args, **kwargs):
        if self.last_goal_dist is not None:
            w = (
                self._pos_factor
                if (self.last_goal_dist - distance_to_goal) > 0
                else self._neg_factor
            )
            self.add_reward(w * (self.last_goal_dist - distance_to_goal))
        self.last_goal_dist = distance_to_goal


class RewardCollision(RewardUnit):
    def __init__(
        self, reward_function: RewardFunction, reward: float = -10.0, *args, **kwargs
    ):
        self._reward = reward
        super().__init__(reward_function, *args, **kwargs)

    def check_parameters(self, *args, **kwargs):
        if self._reward > 0.0:
            warn_msg = (
                f"[{self.__class__.__name__}] Reconsider this reward. "
                f"Positive rewards may lead to unfavorable behaviors. "
                f"Current value: {self._reward}"
            )
            warn(warn_msg)

    def __call__(self, laser_scan: np.ndarray, *args: Any, **kwargs: Any) -> Any:
        coll_in_blind_spots = False
        if "full_laser_scan" in kwargs:
            coll_in_blind_spots = kwargs["full_laser_scan"].min() <= self.robot_radius

        if laser_scan.min() <= self.robot_radius or coll_in_blind_spots:
            self.add_reward(self._reward)
            self.add_info({"is_done": True, "done_reason": 1, "is_success": False})


class RewardDistanceTravelled(RewardUnit):
    def __init__(
        self,
        reward_function: RewardFunction,
        consumption_factor: float = 0.005,
        lin_vel_scalar: float = 1.0,
        ang_vel_scalar: float = 0.001,
        *args,
        **kwargs,
    ):
        self._factor = consumption_factor
        self._lin_vel_scalar = lin_vel_scalar
        self._ang_vel_scalar = ang_vel_scalar
        super().__init__(reward_function, *args, **kwargs)

    def __call__(self, action: np.ndarray, *args: Any, **kwargs: Any) -> Any:
        if action is None:
            pass
        else:
            lin_vel, ang_vel = action[0], action[-1]
            reward = (
                (lin_vel * self._lin_vel_scalar) + (ang_vel * self._ang_vel_scalar)
            ) * -self._factor
            self.add_reward(reward)


class RewardApproachGlobalplan(RewardUnitGlobalplan):
    def __init__(
        self,
        reward_function: RewardFunction,
        pos_factor: float = 0.3,
        neg_factor: float = 0.5,
        on_safe_dist_violation: bool = False,
        *args,
        **kwargs,
    ):
        self._pos_factor = pos_factor
        self._neg_factor = neg_factor
        self._on_safe_dist_violation = on_safe_dist_violation

        super().__init__(reward_function, *args, **kwargs)

        self.last_dist_to_path = None
        self._kdtree = None

    def check_parameters(self, *args, **kwargs):
        if self._pos_factor < 0 or self._neg_factor:
            warn_msg = (
                f"[{self.__class__.__name__}] Both factors should be positive. "
                f"Current values: [pos_factor={self._pos_factor}], [neg_factor={self._neg_factor}]"
            )
            warn(warn_msg)
        if self._pos_factor >= self._neg_factor:
            warn_msg = (
                "'pos_factor' should be smaller than 'neg_factor' otherwise rotary trajectories will get rewarded. "
                f"Current values: [pos_factor={self._pos_factor}], [neg_factor={self._neg_factor}]"
            )
            warn(warn_msg)

    def __call__(
        self, global_plan: np.ndarray, robot_pose, *args: Any, **kwargs: Any
    ) -> Any:
        if not self.curr_dist_to_path and global_plan and len(global_plan) > 0:
            self.curr_dist_to_path = self.get_dist_to_globalplan(
                global_plan, robot_pose
            )

        if self._on_safe_dist_violation:
            if self.curr_dist_to_path and self.last_dist_to_path is not None:
                self.add_reward(self._calc_reward())
        else:
            if (
                not self.safe_dist_breached
                and self.curr_dist_to_path
                and self.last_dist_to_path is not None
            ):
                self.add_reward(self._calc_reward())

        self.last_dist_to_path = self.curr_dist_to_path

    def _calc_reward(self) -> float:
        w = (
            self._pos_factor
            if self.curr_dist_to_path < self.last_dist_to_path
            else self._neg_factor
        )
        return w * (self.last_dist_to_path - self.curr_dist_to_path)

    def reset(self):
        self._kdtree = None
        self.last_dist_to_path = None
        self.curr_dist_to_path = None


class RewardFollowGlobalplan(RewardUnitGlobalplan):
    def __init__(
        self,
        reward_function: RewardFunction,
        min_dist_to_path: float = 0.5,
        reward_factor: float = 0.1,
        *args,
        **kwargs,
    ) -> None:
        self._min_dist_to_path = min_dist_to_path
        self._reward_factor = reward_factor
        super().__init__(reward_function, *args, **kwargs)

    def __call__(
        self,
        action: np.ndarray,
        global_plan: np.ndarray,
        robot_pose,
        *args: Any,
        **kwargs: Any,
    ) -> Any:
        if not self.curr_dist_to_path and global_plan and len(global_plan) > 0:
            self.curr_dist_to_path = self.get_dist_to_globalplan(
                global_plan, robot_pose
            )

        if (
            self.curr_dist_to_path
            and action is not None
            and self.curr_dist_to_path <= self.curr_dist_to_path
        ):
            self.add_reward(self._reward_factor * action[0])


class RewardReverseDrive(RewardUnit):
    def __init__(
        self, reward_function: RewardFunction, reward: float = 0.01, *args, **kwargs
    ) -> None:
        self._reward = reward
        super().__init__(reward_function, *args, **kwargs)

    def __call__(self, action: np.ndarray, *args, **kwargs):
        if action is not None and action[0] < 0:
            self.add_reward(-self._reward)


class RewardActionAbruptVelocityChange(RewardUnit):
    pass
