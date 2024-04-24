import random
from typing import Any, Callable, Dict
from warnings import warn

import numpy as np
import rospy
from rl_utils.utils.observation_collector.constants import DONE_REASONS, OBS_DICT_KEYS

from ..constants import DEFAULTS, REWARD_CONSTANTS
from ..reward_function import RewardFunction
from ..utils import check_params, get_ped_type_min_distances
from .base_reward_units import GlobalplanRewardUnit, RewardUnit
from .reward_unit_factory import RewardUnitFactory

# UPDATE WHEN ADDING A NEW UNIT
__all__ = [
    "RewardGoalReached",
    "RewardSafeDistance",
    "RewardNoMovement",
    "RewardApproachGoal",
    "RewardCollision",
    "RewardDistanceTravelled",
    "RewardApproachGlobalplan",
    "RewardFollowGlobalplan",
    "RewardReverseDrive",
    "RewardAbruptVelocityChange",
    "RewardRootVelocityDifference",
    "RewardTwoFactorVelocityDifference",
    "RewardActiveHeadingDirection",
]


@RewardUnitFactory.register("goal_reached")
class RewardGoalReached(RewardUnit):
    DONE_INFO = {
        "is_done": True,
        "done_reason": DONE_REASONS.SUCCESS.name,
        "is_success": True,
    }
    NOT_DONE_INFO = {"is_done": False}

    @check_params
    def __init__(
        self,
        reward_function: RewardFunction,
        reward: float = DEFAULTS.GOAL_REACHED.REWARD,
        _on_safe_dist_violation: bool = DEFAULTS.GOAL_REACHED._ON_SAFE_DIST_VIOLATION,
        *args,
        **kwargs,
    ):
        """Class for calculating the reward when the goal is reached.

        Args:
            reward_function (RewardFunction): The reward function object holding this unit.
            reward (float, optional): The reward value for reaching the goal. Defaults to DEFAULTS.GOAL_REACHED.REWARD.
            _on_safe_dist_violation (bool, optional): Flag to indicate if there is a violation of safe distance. Defaults to DEFAULTS.GOAL_REACHED._ON_SAFE_DIST_VIOLATION.
        """
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)
        self._reward = reward
        self._goal_radius = self._reward_function.goal_radius

    def check_parameters(self, *args, **kwargs):
        if self._reward < 0.0:
            warn_msg = (
                f"[{self.__class__.__name__}] Reconsider this reward. "
                f"Negative rewards may lead to unfavorable behaviors. "
                f"Current value: {self._reward}"
            )
            warn(warn_msg)

    def __call__(self, distance_to_goal: float, *args: Any, **kwargs: Any) -> None:
        """Calculates the reward and updates the information when the goal is reached.

        Args:
            distance_to_goal (float): Distance to the goal in m.
        """
        if distance_to_goal < self._reward_function.goal_radius:
            self.add_reward(self._reward)
            self.add_info(self.DONE_INFO)
        else:
            self.add_info(self.NOT_DONE_INFO)

    def reset(self):
        self._goal_radius = self._reward_function.goal_radius


@RewardUnitFactory.register("safe_distance")
class RewardSafeDistance(RewardUnit):
    SAFE_DIST_VIOLATION_INFO = {"safe_dist_violation": True}

    @check_params
    def __init__(
        self,
        reward_function: RewardFunction,
        reward: float = DEFAULTS.SAFE_DISTANCE.REWARD,
        *args,
        **kwargs,
    ):
        """Class for calculating the reward when violating the safe distance.

        Args:
            reward_function (RewardFunction): The reward function object.
            reward (float, optional): The reward value for violating the safe distance. Defaults to DEFAULTS.SAFE_DISTANCE.REWARD.
        """
        super().__init__(reward_function, True, *args, **kwargs)
        self._reward = reward
        self._safe_dist = self._reward_function._safe_dist

    def check_parameters(self, *args, **kwargs):
        if self._reward > 0.0:
            warn_msg = (
                f"[{self.__class__.__name__}] Reconsider this reward. "
                f"Positive rewards may lead to unfavorable behaviors. "
                f"Current value: {self._reward}"
            )
            warn(warn_msg)

    def __call__(self, *args: Any, **kwargs: Any):
        violation_in_blind_spot = False
        if "full_laser_scan" in kwargs and len(kwargs["full_laser_scan"]) > 0:
            violation_in_blind_spot = kwargs["full_laser_scan"].min() <= self._safe_dist

        if (
            self.get_internal_state_info("safe_dist_breached")
            or violation_in_blind_spot
        ):
            self.add_reward(self._reward)
            self.add_info(self.SAFE_DIST_VIOLATION_INFO)


@RewardUnitFactory.register("no_movement")
class RewardNoMovement(RewardUnit):
    @check_params
    def __init__(
        self,
        reward_function: RewardFunction,
        reward: float = DEFAULTS.NO_MOVEMENT.REWARD,
        _on_safe_dist_violation: bool = DEFAULTS.NO_MOVEMENT._ON_SAFE_DIST_VIOLATION,
        *args,
        **kwargs,
    ):
        """Class for calculating the reward when there is no movement.

        Args:
            reward_function (RewardFunction): The reward function object.
            reward (float, optional): The reward value for no movement. Defaults to DEFAULTS.NO_MOVEMENT.REWARD.
            _on_safe_dist_violation (bool, optional): Flag to indicate if there is a violation of safe distance. Defaults to DEFAULTS.NO_MOVEMENT._ON_SAFE_DIST_VIOLATION.
        """
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)
        self._reward = reward

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


@RewardUnitFactory.register("approach_goal")
class RewardApproachGoal(RewardUnit):
    @check_params
    def __init__(
        self,
        reward_function: RewardFunction,
        pos_factor: float = DEFAULTS.APPROACH_GOAL.POS_FACTOR,
        neg_factor: float = DEFAULTS.APPROACH_GOAL.NEG_FACTOR,
        _on_safe_dist_violation: bool = DEFAULTS.APPROACH_GOAL._ON_SAFE_DIST_VIOLATION,
        *args,
        **kwargs,
    ):
        """Class for calculating the reward when approaching the goal.

        Args:
            reward_function (RewardFunction): The reward function object.
            pos_factor (float, optional): Positive factor for approaching the goal. Defaults to DEFAULTS.APPROACH_GOAL.POS_FACTOR.
            neg_factor (float, optional): Negative factor for distancing from the goal. Defaults to DEFAULTS.APPROACH_GOAL.NEG_FACTOR.
            _on_safe_dist_violation (bool, optional): Flag to indicate if there is a violation of safe distance. Defaults to DEFAULTS.APPROACH_GOAL._ON_SAFE_DIST_VIOLATION.
        """
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)
        self._pos_factor = pos_factor
        self._neg_factor = neg_factor
        self.last_goal_dist = None

    def check_parameters(self, *args, **kwargs):
        if self._pos_factor < 0 or self._neg_factor < 0:
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

    def reset(self):
        self.last_goal_dist = None


@RewardUnitFactory.register("collision")
class RewardCollision(RewardUnit):
    DONE_INFO = {
        "is_done": True,
        "done_reason": DONE_REASONS.COLLISION.name,
        "is_success": False,
    }

    @check_params
    def __init__(
        self,
        reward_function: RewardFunction,
        reward: float = DEFAULTS.COLLISION.REWARD,
        bumper_zone: float = DEFAULTS.COLLISION.BUMPER_ZONE,
        *args,
        **kwargs,
    ):
        """Class for calculating the reward when a collision is detected.

        Args:
            reward_function (RewardFunction): The reward function object.
            reward (float, optional): The reward value for reaching the goal. Defaults to DEFAULTS.COLLISION.REWARD.
        """
        super().__init__(reward_function, True, *args, **kwargs)
        self._reward = reward
        self._bumper_zone = bumper_zone + self.robot_radius

    def check_parameters(self, *args, **kwargs):
        if self._reward > 0.0:
            warn_msg = (
                f"[{self.__class__.__name__}] Reconsider this reward. "
                f"Positive rewards may lead to unfavorable behaviors. "
                f"Current value: {self._reward}"
            )
            warn(warn_msg)

    def __call__(self, *args: Any, **kwargs: Any) -> Any:
        coll_in_blind_spots = False
        if "full_laser_scan" in kwargs:
            if len(kwargs["full_laser_scan"]) > 0:
                coll_in_blind_spots = (
                    kwargs["full_laser_scan"].min() <= self._bumper_zone
                )

        laser_min = self.get_internal_state_info("min_dist_laser")
        if laser_min <= self.robot_radius or coll_in_blind_spots:
            self.add_reward(self._reward)
            self.add_info(self.DONE_INFO)


@RewardUnitFactory.register("distance_travelled")
class RewardDistanceTravelled(RewardUnit):
    def __init__(
        self,
        reward_function: RewardFunction,
        consumption_factor: float = DEFAULTS.DISTANCE_TRAVELLED.CONSUMPTION_FACTOR,
        lin_vel_scalar: float = DEFAULTS.DISTANCE_TRAVELLED.LIN_VEL_SCALAR,
        ang_vel_scalar: float = DEFAULTS.DISTANCE_TRAVELLED.ANG_VEL_SCALAR,
        _on_safe_dist_violation: bool = DEFAULTS.DISTANCE_TRAVELLED._ON_SAFE_DIST_VIOLATION,
        *args,
        **kwargs,
    ):
        """Class for calculating the reward for the distance travelled.

        Args:
            reward_function (RewardFunction): The reward function object.
            consumption_factor (float, optional): Negative consumption factor. Defaults to DEFAULTS.DISTANCE_TRAVELLED.CONSUMPTION_FACTOR.
            lin_vel_scalar (float, optional): Scalar for the linear velocity. Defaults to DEFAULTS.DISTANCE_TRAVELLED.LIN_VEL_SCALAR.
            ang_vel_scalar (float, optional): Scalar for the angular velocity. Defaults to DEFAULTS.DISTANCE_TRAVELLED.ANG_VEL_SCALAR.
            _on_safe_dist_violation (bool, optional): Flag to indicate if there is a violation of safe distance. Defaults to DEFAULTS.DISTANCE_TRAVELLED._ON_SAFE_DIST_VIOLATION.
        """
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)
        self._factor = consumption_factor
        self._lin_vel_scalar = lin_vel_scalar
        self._ang_vel_scalar = ang_vel_scalar

    def __call__(self, action: np.ndarray, *args: Any, **kwargs: Any) -> Any:
        if action is None:
            return
        lin_vel, ang_vel = action[0], action[-1]
        reward = (
            (lin_vel * self._lin_vel_scalar) + (ang_vel * self._ang_vel_scalar)
        ) * -self._factor
        self.add_reward(reward)


@RewardUnitFactory.register("approach_globalplan")
class RewardApproachGlobalplan(GlobalplanRewardUnit):
    @check_params
    def __init__(
        self,
        reward_function: RewardFunction,
        pos_factor: float = DEFAULTS.APPROACH_GLOBALPLAN.POS_FACTOR,
        neg_factor: float = DEFAULTS.APPROACH_GLOBALPLAN.NEG_FACTOR,
        _on_safe_dist_violation: bool = DEFAULTS.APPROACH_GLOBALPLAN._ON_SAFE_DIST_VIOLATION,
        *args,
        **kwargs,
    ):
        """Class for calculating the reward for approaching the global plan.

        Args:
            reward_function (RewardFunction): The reward function object.
            pos_factor (float, optional): Positive factor for approaching the goal. Defaults to DEFAULTS.APPROACH_GLOBALPLAN.POS_FACTOR.
            neg_factor (float, optional): Negative factor for distancing from the goal. Defaults to DEFAULTS.APPROACH_GLOBALPLAN.NEG_FACTOR.
            _on_safe_dist_violation (bool, optional): Flag to indicate if there is a violation of safe distance. Defaults to DEFAULTS.APPROACH_GLOBALPLAN._ON_SAFE_DIST_VIOLATION.
        """
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)

        self._pos_factor = pos_factor
        self._neg_factor = neg_factor

        self.last_dist_to_path = None
        self._kdtree = None

    def check_parameters(self, *args, **kwargs):
        if self._pos_factor < 0 or self._neg_factor < 0:
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
        super().__call__(global_plan=global_plan, robot_pose=robot_pose)

        if self.curr_dist_to_path and self.last_dist_to_path:
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
        super().reset()
        self.last_dist_to_path = None


@RewardUnitFactory.register("follow_globalplan")
class RewardFollowGlobalplan(GlobalplanRewardUnit):
    """
    RewardFollowGlobalplan is a reward unit that calculates the reward based on the agent's
    distance to the global plan and its action.

    Args:
        reward_function (RewardFunction): The reward function to use for calculating the reward.
        min_dist_to_path (float, optional): The minimum distance to the global plan. Defaults to DEFAULTS.FOLLOW_GLOBALPLAN.MIN_DIST_TO_PATH.
        reward_factor (float, optional): The reward factor to multiply the action by. Defaults to DEFAULTS.FOLLOW_GLOBALPLAN.REWARD_FACTOR.
        _on_safe_dist_violation (bool, optional): Flag indicating whether to handle safe distance violations. Defaults to DEFAULTS.FOLLOW_GLOBALPLAN._ON_SAFE_DIST_VIOLATION.
        *args: Variable length argument list.
        **kwargs: Arbitrary keyword arguments.

    Attributes:
        _min_dist_to_path (float): The minimum distance to the global plan.
        _reward_factor (float): The reward factor to multiply the action by.

    Methods:
        __call__(self, action, global_plan, robot_pose, *args, **kwargs): Calculates the reward based on the agent's distance to the global plan and its action.
    """

    def __init__(
        self,
        reward_function: RewardFunction,
        min_dist_to_path: float = DEFAULTS.FOLLOW_GLOBALPLAN.MIN_DIST_TO_PATH,
        reward_factor: float = DEFAULTS.FOLLOW_GLOBALPLAN.REWARD_FACTOR,
        _on_safe_dist_violation: bool = DEFAULTS.FOLLOW_GLOBALPLAN._ON_SAFE_DIST_VIOLATION,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)

        self._min_dist_to_path = min_dist_to_path
        self._reward_factor = reward_factor

    def __call__(
        self,
        action: np.ndarray,
        global_plan: np.ndarray,
        robot_pose,
        *args: Any,
        **kwargs: Any,
    ) -> Any:
        """
        Calculates the reward based on the given action, global plan, and robot pose.

        Args:
            action (np.ndarray): The action taken by the agent.
            global_plan (np.ndarray): The global plan for the robot.
            robot_pose: The current pose of the robot.
            *args: Additional positional arguments.
            **kwargs: Additional keyword arguments.

        Returns:
            Any: The calculated reward.
        """
        super().__call__(global_plan=global_plan, robot_pose=robot_pose)

        if (
            self.curr_dist_to_path
            and action is not None
            and self.curr_dist_to_path <= self._min_dist_to_path
        ):
            self.add_reward(self._reward_factor * action[0])


@RewardUnitFactory.register("reverse_drive")
class RewardReverseDrive(RewardUnit):
    """
    A reward unit that provides a reward for driving in reverse.

    Args:
        reward_function (RewardFunction): The reward function to be used.
        reward (float, optional): The reward value for driving in reverse. Defaults to DEFAULTS.REVERSE_DRIVE.REWARD.
        _on_safe_dist_violation (bool, optional): Whether to penalize for violating safe distance. Defaults to DEFAULTS.REVERSE_DRIVE._ON_SAFE_DIST_VIOLATION.
        *args: Variable length argument list.
        **kwargs: Arbitrary keyword arguments.

    Attributes:
        _reward (float): The reward value for driving in reverse.

    Methods:
        check_parameters: Checks if the reward value is positive and issues a warning if it is.
        __call__: Adds the reward value to the total reward if the action is not None and the first element of the action is less than 0.

    """

    @check_params
    def __init__(
        self,
        reward_function: RewardFunction,
        reward: float = DEFAULTS.REVERSE_DRIVE.REWARD,
        _on_safe_dist_violation: bool = DEFAULTS.REVERSE_DRIVE._ON_SAFE_DIST_VIOLATION,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)

        self._reward = reward

    def check_parameters(self, *args, **kwargs):
        """
        Checks if the reward value is positive and issues a warning if it is.
        """
        if self._reward > 0.0:
            warn_msg = (
                f"[{self.__class__.__name__}] Reconsider this reward. "
                f"Positive rewards may lead to unfavorable behaviors. "
                f"Current value: {self._reward}"
            )
            warn(warn_msg)

    def __call__(self, action: np.ndarray, *args, **kwargs):
        """
        Adds the reward value to the total reward if the action is not None and the first element of the action is less than 0.

        Args:
            action (np.ndarray): The action taken.

        """
        if action is not None and action[0] < 0:
            self.add_reward(self._reward)


@RewardUnitFactory.register("abrupt_velocity_change")
class RewardAbruptVelocityChange(RewardUnit):
    """
    A reward unit that penalizes abrupt changes in velocity.

    Args:
        reward_function (RewardFunction): The reward function to be used.
        vel_factors (Dict[str, float], optional): Velocity factors for each dimension. Defaults to DEFAULTS.ABRUPT_VEL_CHANGE.VEL_FACTORS.
        _on_safe_dist_violation (bool, optional): Flag indicating whether to penalize abrupt velocity changes on safe distance violation. Defaults to DEFAULTS.ABRUPT_VEL_CHANGE._ON_SAFE_DIST_VIOLATION.

    Attributes:
        _vel_factors (Dict[str, float]): Velocity factors for each dimension.
        last_action (np.ndarray): The last action taken.
        _vel_change_fcts (List[Callable[[np.ndarray], None]]): List of velocity change functions.

    Methods:
        _get_vel_change_fcts(): Returns a list of velocity change functions.
        _prepare_reward_function(idx: int, factor: float) -> Callable[[np.ndarray], None]: Prepares a reward function for a specific dimension.
        __call__(action: np.ndarray, *args, **kwargs): Calculates the reward based on the action taken.
        reset(): Resets the last action to None.
    """

    def __init__(
        self,
        reward_function: RewardFunction,
        vel_factors: Dict[str, float] = DEFAULTS.ABRUPT_VEL_CHANGE.VEL_FACTORS,
        _on_safe_dist_violation: bool = DEFAULTS.ABRUPT_VEL_CHANGE._ON_SAFE_DIST_VIOLATION,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)

        self._vel_factors = vel_factors
        self.last_action = None

        self._vel_change_fcts = self._get_vel_change_fcts()

    def _get_vel_change_fcts(self):
        return [
            self._prepare_reward_function(int(idx), factor)
            for idx, factor in self._vel_factors.items()
        ]

    def _prepare_reward_function(
        self, idx: int, factor: float
    ) -> Callable[[np.ndarray], None]:
        def vel_change_fct(action: np.ndarray):
            assert isinstance(self.last_action, np.ndarray)
            vel_diff = abs(action[idx] - self.last_action[idx])
            self.add_reward(-((vel_diff**4 / 100) * factor))

        return vel_change_fct

    def __call__(self, action: np.ndarray, *args, **kwargs):
        if self.last_action is not None:
            for rew_fct in self._vel_change_fcts:
                rew_fct(action)
        self.last_action = action

    def reset(self):
        self.last_action = None


@RewardUnitFactory.register("root_velocity_difference")
class RewardRootVelocityDifference(RewardUnit):
    """
    A reward unit that calculates the difference in root velocity between consecutive actions.

    Args:
        reward_function (RewardFunction): The reward function to be used.
        k (float, optional): The scaling factor for the velocity difference. Defaults to DEFAULTS.ROOT_VEL_DIFF.K.
        _on_safe_dist_violation (bool, optional): Flag indicating whether to penalize for violating safe distance.
            Defaults to DEFAULTS.ROOT_VEL_DIFF._ON_SAFE_DIST_VIOLATION.
        *args: Variable length argument list.
        **kwargs: Arbitrary keyword arguments.

    Attributes:
        _k (float): The scaling factor for the velocity difference.
        last_action (numpy.ndarray): The last action taken.

    Methods:
        __call__(self, action: np.ndarray, *args, **kwargs): Calculates the reward based on the velocity difference between
            the current action and the last action.
        reset(self): Resets the last action to None.
    """

    def __init__(
        self,
        reward_function: RewardFunction,
        k: float = DEFAULTS.ROOT_VEL_DIFF.K,
        _on_safe_dist_violation: bool = DEFAULTS.ROOT_VEL_DIFF._ON_SAFE_DIST_VIOLATION,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)

        self._k = k
        self.last_action = None

    def __call__(self, action: np.ndarray, *args, **kwargs):
        """
        Calculates and adds the reward based on the given action.

        Args:
            action (np.ndarray): The action taken by the agent.

        Returns:
            None
        """
        if self.last_action is not None:
            vel_diff = np.linalg.norm((action - self.last_action) ** 2)
            if vel_diff < self._k:
                self.add_reward((1 - vel_diff) / self._k)
        self.last_action = action

    def reset(self):
        self.last_action = None


@RewardUnitFactory.register("two_factor_velocity_difference")
class RewardTwoFactorVelocityDifference(RewardUnit):
    """
    A reward unit that calculates the difference in velocity between consecutive actions
    and penalizes the agent based on the squared difference.

    Args:
        reward_function (RewardFunction): The reward function to be used.
        alpha (float, optional): The weight for the squared difference in the first dimension of the action. Defaults to DEFAULTS.ROOT_VEL_DIFF.K.
        beta (float, optional): The weight for the squared difference in the last dimension of the action. Defaults to DEFAULTS.ROOT_VEL_DIFF.K.
        _on_safe_dist_violation (bool, optional): Flag indicating whether to penalize the agent on safe distance violation. Defaults to DEFAULTS.ROOT_VEL_DIFF._ON_SAFE_DIST_VIOLATION.
        *args: Variable length argument list.
        **kwargs: Arbitrary keyword arguments.
    """

    def __init__(
        self,
        reward_function: RewardFunction,
        alpha: float = DEFAULTS.TWO_FACTOR_VEL_DIFF.ALPHA,
        beta: float = DEFAULTS.TWO_FACTOR_VEL_DIFF.BETA,
        _on_safe_dist_violation: bool = DEFAULTS.ROOT_VEL_DIFF._ON_SAFE_DIST_VIOLATION,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)

        self._alpha = alpha
        self._beta = beta
        self.last_action = None

    def __call__(self, action: np.ndarray, *args, **kwargs):
        """
        Calculates and adds the reward based on the difference between the current action and the last action.

        Args:
            action (np.ndarray): The current action.

        Returns:
            None
        """
        if self.last_action is not None:
            diff = (action - self.last_action) ** 2
            self.add_reward(-(diff[0] * self._alpha + diff[-1] * self._beta))
        self.last_action = action

    def reset(self):
        self.last_action = None


@RewardUnitFactory.register("active_heading_direction")
class RewardActiveHeadingDirection(RewardUnit):
    """
    Reward unit that calculates the reward based on the active heading direction of the robot.

    Args:
        reward_function (RewardFunction): The reward function to be used.
        r_angle (float, optional): Weight for difference between max deviation of heading direction and desired heading direction. Defaults to 0.6.
        theta_m (float, optional): Maximum allowable deviation of the heading direction. Defaults to np.pi/6.
        theta_min (int, optional): Minimum allowable deviation of the heading direction. Defaults to 1000.
        ped_min_dist (float, optional): Minimum distance to pedestrians. Defaults to 8.0.
        iters (int, optional): Number of iterations to find a reachable available theta. Defaults to 60.
        _on_safe_dist_violation (bool, optional): Flag indicating whether to penalize the reward on safe distance violation. Defaults to True.
        *args: Variable length argument list.
        **kwargs: Arbitrary keyword arguments.

    Attributes:
        _r_angle (float): Desired heading direction in the robot's local frame.
        _theta_m (float): Maximum allowable deviation of the heading direction.
        _theta_min (int): Minimum allowable deviation of the heading direction.
        _ped_min_dist (float): Minimum application distance to pedestrians.
        _iters (int): Number of iterations to find a reachable available theta.
    """

    def __init__(
        self,
        reward_function: RewardFunction,
        r_angle: float = 0.6,
        theta_m: float = np.pi / 6,
        theta_min: int = 1000,
        ped_min_dist: float = 8.0,
        iters: int = 60,
        _on_safe_dist_violation: bool = True,
        *args,
        **kwargs,
    ) -> None:
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)
        self._r_angle = r_angle
        self._theta_m = theta_m
        self._theta_min = theta_min
        self._ped_min_dist = ped_min_dist
        self._iters = iters

    def __call__(
        self,
        goal_in_robot_frame: np.ndarray,
        action: np.ndarray,
        relative_location: np.ndarray,
        relative_x_vel: np.ndarray,
        relative_y_vel: np.ndarray,
        *args,
        **kwargs,
    ) -> float:
        """
        Calculates the reward based on the active heading direction of the robot.

        Args:
            goal_in_robot_frame (np.ndarray): The goal position in the robot's frame of reference.
            action (np.ndarray): The last action taken by the robot.
            relative_location (np.ndarray): The relative location of the pedestrians.
            relative_x_vel (np.ndarray): The relative x-velocity of the pedestrians.
            relative_y_vel (np.ndarray): The relative y-velocity of the pedestrians.
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.

        Returns:
            float: The calculated reward based on the active heading direction.
        """
        if (
            relative_location is None
            or relative_x_vel is None
            or relative_y_vel is None
        ):
            return 0.0

        # prefer goal theta:
        theta_pre = goal_in_robot_frame[1]
        d_theta = theta_pre

        v_x = action[0]

        # get the pedestrian's position:
        if len(relative_location) != 0:  # tracker results
            d_theta = np.pi / 2  # theta_pre
            theta_min = self._theta_min
            for _ in range(self._iters):
                theta = random.uniform(-np.pi, np.pi)
                free = True
                for ped_location, ped_x_vel, ped_y_vel in zip(
                    relative_location, relative_x_vel, relative_y_vel
                ):
                    p_x = ped_location[0]
                    p_y = ped_location[1]
                    p_vx = ped_x_vel
                    p_vy = ped_y_vel

                    ped_dis = np.linalg.norm([p_x, p_y])

                    if ped_dis <= self._ped_min_dist:
                        ped_theta = np.arctan2(p_y, p_x)

                        # 3*robot_radius:= estimation for sum of the pedestrian radius and the robot radius
                        vector = ped_dis**2 - (3 * self.robot_radius) ** 2
                        if vector < 0:
                            continue  # in this case the robot likely crashed into the pedestrian, disregard this pedestrian

                        vo_theta = np.arctan2(
                            3 * self.robot_radius,
                            np.sqrt(vector),
                        )
                        # Check if the robot's trajectory intersects with the pedestrian's VO cone
                        theta_rp = np.arctan2(
                            v_x * np.sin(theta) - p_vy, v_x * np.cos(theta) - p_vx
                        )
                        if theta_rp >= (ped_theta - vo_theta) and theta_rp <= (
                            ped_theta + vo_theta
                        ):
                            free = False
                            break

                # Find the reachable available theta that minimizes the difference from the goal theta
                if free:
                    theta_diff = (theta - theta_pre) ** 2
                    if theta_diff < theta_min:
                        theta_min = theta_diff
                        d_theta = theta

        else:  # no obstacles:
            d_theta = theta_pre

        return self._r_angle * (self._theta_m - abs(d_theta))


@RewardUnitFactory.register("ped_type_safety_distance")
class RewardPedTypeSafetyDistance(RewardUnit):
    """
    RewardPedTypeDistance is a reward unit that provides a reward based on the distance between the agent and a specific pedestrian type.

    Args:
        reward_function (RewardFunction): The reward function to which this reward unit belongs.
        ped_type (int, optional): The type of pedestrian to consider. Defaults to DEFAULTS.PED_TYPE_SPECIFIC_SAFETY_DISTANCE.TYPE.
        reward (float, optional): The reward value to be added if the distance to the pedestrian type is less than the safety distance. Defaults to DEFAULTS.PED_TYPE_SPECIFIC_SAFETY_DISTANCE.REWARD.
        safety_distance (float, optional): The safety distance threshold. If the distance to the pedestrian type is less than this value, the reward is added. Defaults to DEFAULTS.PED_TYPE_SPECIFIC_SAFETY_DISTANCE.DISTANCE.
        _on_safe_dist_violation (bool, optional): A flag indicating whether to trigger a violation event when the safety distance is violated. Defaults to DEFAULTS.PED_TYPE_SPECIFIC_SAFETY_DISTANCE._ON_SAFE_DIST_VIOLATION.
        *args: Variable length argument list.
        **kwargs: Arbitrary keyword arguments.

    Attributes:
        _type (int): The type of pedestrian to consider.
        _reward (float): The reward value to be added if the distance to the pedestrian type is less than the safety distance.
        _safety_distance (float): The safety distance threshold.

    Methods:
        __call__(*args, **kwargs): Calculates the reward based on the distance to the pedestrian type.
        reset(): Resets the reward unit.
    """

    def __init__(
        self,
        reward_function: RewardFunction,
        ped_type: int = DEFAULTS.PED_TYPE_SPECIFIC_SAFETY_DISTANCE.TYPE,
        reward: float = DEFAULTS.PED_TYPE_SPECIFIC_SAFETY_DISTANCE.REWARD,
        safety_distance: float = DEFAULTS.PED_TYPE_SPECIFIC_SAFETY_DISTANCE.DISTANCE,
        _on_safe_dist_violation: bool = DEFAULTS.PED_TYPE_SPECIFIC_SAFETY_DISTANCE._ON_SAFE_DIST_VIOLATION,
        *args,
        **kwargs,
    ):
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)
        self._type = ped_type
        self._reward = reward
        self._safety_distance = safety_distance

    def __call__(self, *args: Any, **kwargs: Any) -> None:
        ped_type_min_distances = self.get_internal_state_info(
            "min_distances_per_ped_type"
        )

        if ped_type_min_distances is None:
            self.add_internal_state_info(
                key="min_distances_per_ped_type",
                value=get_ped_type_min_distances(**kwargs),
            )
            ped_type_min_distances = self.get_internal_state_info(
                "min_distances_per_ped_type"
            )

        if self._type not in ped_type_min_distances:
            rospy.logwarn_throttle(
                60,
                f"[{rospy.get_name()}, {self.__class__.__name__}] Pedestrian type {self._type} not found.",
            )
            return

        if ped_type_min_distances[self._type] < self._safety_distance:
            self.add_reward(self._reward)

    def reset(self):
        pass


@RewardUnitFactory.register("ped_type_collision")
class RewardPedTypeCollision(RewardUnit):
    """
    RewardPedTypeCollision is a reward unit that provides a reward when the robot collides with a specific pedestrian type.

    Args:
        reward_function (RewardFunction): The reward function to which this reward unit belongs.
        ped_type (int, optional): The specific pedestrian type to check for collision. Defaults to DEFAULTS.PED_TYPE_SPECIFIC_COLLISION.TYPE.
        reward (float, optional): The reward value to be added when a collision with the specific pedestrian type occurs. Defaults to DEFAULTS.PED_TYPE_SPECIFIC_COLLISION.REWARD.
        *args: Variable length argument list.
        **kwargs: Arbitrary keyword arguments.

    Attributes:
        _type (int): The specific pedestrian type to check for collision.
        _reward (float): The reward value to be added when a collision with the specific pedestrian type occurs.
    """

    def __init__(
        self,
        reward_function: RewardFunction,
        ped_type: int = DEFAULTS.PED_TYPE_SPECIFIC_COLLISION.TYPE,
        reward: float = DEFAULTS.PED_TYPE_SPECIFIC_COLLISION.REWARD,
        bumper_zone: float = DEFAULTS.PED_TYPE_SPECIFIC_COLLISION.BUMPER_ZONE,
        *args,
        **kwargs,
    ):
        super().__init__(reward_function, True, *args, **kwargs)
        self._type = ped_type
        self._reward = reward
        self._bumper_zone = self.robot_radius + bumper_zone

    def __call__(self, *args: Any, **kwargs: Any) -> None:
        """
        Checks if the robot has collided with the specific pedestrian type and adds the reward if a collision occurs.

        Args:
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.
        """
        ped_type_min_distances = self.get_internal_state_info(
            "min_distances_per_ped_type"
        )

        if ped_type_min_distances is None:
            self.add_internal_state_info(
                key="min_distances_per_ped_type",
                value=get_ped_type_min_distances(**kwargs),
            )
            ped_type_min_distances = self.get_internal_state_info(
                "min_distances_per_ped_type"
            )

        if self._type not in ped_type_min_distances:
            rospy.logwarn_throttle(
                60,
                f"[{rospy.get_name()}, {self.__class__.__name__}] Pedestrian type {self._type} not found.",
            )
            return

        if ped_type_min_distances[self._type] <= self._bumper_zone:
            self.add_reward(self._reward)

    def reset(self):
        pass


@RewardUnitFactory.register("ped_type_vel_constraint")
class RewardPedTypeVelocityConstraint(RewardUnit):

    def __init__(
        self,
        reward_function: RewardFunction,
        ped_type: int = DEFAULTS.PED_TYPE_SPECIFIC_SAFETY_DISTANCE.TYPE,
        penalty_factor: float = 0.05,
        active_distance: float = DEFAULTS.PED_TYPE_SPECIFIC_SAFETY_DISTANCE.DISTANCE,
        _on_safe_dist_violation: bool = DEFAULTS.PED_TYPE_SPECIFIC_SAFETY_DISTANCE._ON_SAFE_DIST_VIOLATION,
        *args,
        **kwargs,
    ):
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)
        self._type = ped_type
        self._penalty_factor = penalty_factor
        self._active_distance = active_distance

    def __call__(self, action: np.ndarray, *args: Any, **kwargs: Any) -> None:
        ped_type_min_distances = self.get_internal_state_info(
            "min_distances_per_ped_type"
        )

        if ped_type_min_distances is None:
            self.add_internal_state_info(
                key="min_distances_per_ped_type",
                value=get_ped_type_min_distances(**kwargs),
            )
            ped_type_min_distances = self.get_internal_state_info(
                "min_distances_per_ped_type"
            )

        if self._type not in ped_type_min_distances:
            rospy.logwarn_throttle(
                60,
                f"[{rospy.get_name()}, {self.__class__.__name__}] Pedestrian type {self._type} not found.",
            )
            return

        if ped_type_min_distances[self._type] < self._active_distance:
            self.add_reward(-self._penalty_factor * action[0])

    def reset(self):
        pass


from geometry_msgs.msg import Pose2D


@RewardUnitFactory.register("angular_vel_constraint")
class RewardAngularVelocityConstraint(RewardUnit):
    def __init__(
        self,
        reward_function: RewardFunction,
        penalty_factor: float = 0.05,
        threshold: float = None,
        _on_safe_dist_violation: bool = True,
        *args,
        **kwargs,
    ):
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)
        self._penalty_factor = penalty_factor
        self._threshold = threshold

        self._time_step_size = rospy.get_param("step_size")
        self._last_theta = None

    def __call__(self, robot_pose: Pose2D, *args: Any, **kwargs: Any) -> None:
        if self._last_theta is not None:
            rotational_vel = (
                abs(robot_pose.theta - self._last_theta) / self._time_step_size
            )
            if self._threshold and rotational_vel > self._threshold:
                self.add_reward(-self._penalty_factor * rotational_vel)
        self._last_theta = robot_pose.theta

    def reset(self):
        self._last_theta = None


@RewardUnitFactory.register("max_steps_exceeded")
class RewardMaxStepsExceeded(RewardUnit):
    """
    A reward unit that penalizes the agent when the maximum number of steps is exceeded.

    Args:
        reward_function (RewardFunction): The reward function to which this unit belongs.
        penalty (float, optional): The penalty value to be applied when the maximum steps are exceeded. Defaults to 10.
        _on_safe_dist_violation (bool, optional): Whether to apply the penalty on safe distance violation. Defaults to True.
        *args: Variable length argument list.
        **kwargs: Arbitrary keyword arguments.

    Attributes:
        _penalty (float): The penalty value to be applied when the maximum steps are exceeded.
        _steps (int): The current number of steps taken.

    Methods:
        __call__(*args, **kwargs): Updates the step count and applies the penalty if the maximum steps are exceeded.
        reset(): Resets the step count to zero.
    """

    DONE_INFO = {
        "is_done": True,
        "done_reason": DONE_REASONS.STEP_LIMIT.name,
        "is_success": 0,
    }

    @check_params
    def __init__(
        self,
        reward_function: RewardFunction,
        penalty: float = 10,
        _on_safe_dist_violation: bool = True,
        *args,
        **kwargs,
    ):
        super().__init__(reward_function, _on_safe_dist_violation, *args, **kwargs)
        self._penalty = penalty
        self._steps = 0

    def check_parameters(self, *args, **kwargs):
        if self._penalty < 0.0:
            warn_msg = (
                f"[{self.__class__.__name__}] Reconsider this reward. "
                f"The penalty should be a positive value as it is going to be subtracted from the total reward."
                f"Current value: {self._penalty}"
            )
            warn(warn_msg)

    def __call__(self, *args: Any, **kwargs: Any) -> None:
        """
        Updates the step count and applies the penalty if the maximum steps are exceeded.

        Args:
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.
        """
        self._steps += 1
        if self._steps >= self._reward_function.max_steps:
            self.add_reward(-self._penalty)
            self.add_info(self.DONE_INFO)

    def reset(self):
        """
        Resets the step count to zero.
        """
        self._steps = 0
