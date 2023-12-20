import functools
import yaml
from tools.constants import TRAINING_CONSTANTS
import numpy as np


def check_params(fn):
    @functools.wraps(fn)
    def wrapper(self, *args, **kwargs):
        fn(self, *args, **kwargs)
        self.check_parameters()
        return

    return wrapper


def load_rew_fnc(config_name: str) -> dict:
    config_location = TRAINING_CONSTANTS.PATHS.REWARD_FUNCTIONS(config_name)
    with open(config_location, "r", encoding="utf-8") as target:
        config = yaml.load(target, Loader=yaml.FullLoader)
    return config


def min_distance_from_pointcloud(point_cloud: np.ndarray):
    return np.min(distances_from_pointcloud(point_cloud))


def distances_from_pointcloud(point_cloud: np.ndarray):
    return np.sqrt(
        point_cloud["x"] ** 2 + point_cloud["y"] ** 2 + point_cloud["z"] ** 2
    )


class InternalStateInfoUpdate:
    """
    Represents a callable object that updates internal state information in a reward function.

    Args:
        key (str): The key associated with the internal state information.
        func (callable): The function that computes the updated internal state information.

    Attributes:
        key (str): The key associated with the internal state information.
        func (callable): The function that computes the updated internal state information.
    """

    def __init__(self, key: str, func):
        self.key = key
        self.func = func

    def __call__(self, reward_function: "RewardFunction", **kwargs):
        """
        Updates the internal state information in the given reward function.

        Args:
            reward_function (RewardFunction): The reward function to update.
            **kwargs: Additional keyword arguments to pass to the internal state update function.
        """
        reward_function.add_internal_state_info(
            self.key, self.func(reward_function=reward_function, **kwargs)
        )


def min_dist_laser(
    laser_scan: np.ndarray,
    point_cloud: np.ndarray,
    from_aggregate_obs: bool,
    *args,
    **kwargs
):
    if laser_scan is None and not point_cloud:
        raise ValueError("Neither LaserScan nor PointCloud data was provided!")

    if not from_aggregate_obs:
        return laser_scan.min()
    else:
        return min_distance_from_pointcloud(point_cloud)


def safe_dist_breached(reward_function: "RewardFunction", *args, **kwargs) -> None:
    return (
        reward_function.get_internal_state_info("min_dist_laser")
        <= reward_function.safe_dist
    )
