import functools

import numpy as np
import yaml
from rl_utils.utils.observation_collector.constants import OBS_DICT_KEYS
from tools.constants import TRAINING_CONSTANTS


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

    if len(laser_scan) == 0:
        return np.inf

    if not from_aggregate_obs:
        return laser_scan.min()
    else:
        return min_distance_from_pointcloud(point_cloud)


def safe_dist_breached(reward_function: "RewardFunction", *args, **kwargs) -> None:
    return (
        reward_function.get_internal_state_info("min_dist_laser")
        <= reward_function.safe_dist
    )


def get_ped_type_min_distances(**kwargs):
    ped_distances = {}

    relative_locations = kwargs.get(
        OBS_DICT_KEYS.SEMANTIC.RELATIVE_LOCATION.value, None
    )
    pedestrian_types = kwargs.get(OBS_DICT_KEYS.SEMANTIC.PEDESTRIAN_TYPE.value, None)

    if relative_locations is None or pedestrian_types is None:
        return ped_distances

    if len(relative_locations) == 0 or len(pedestrian_types.points) == 0:
        return ped_distances

    for relative_loc, type_data in zip(relative_locations, pedestrian_types.points):
        distance = np.linalg.norm(relative_loc)
        evidence = int(type_data.evidence)

        if evidence not in ped_distances or ped_distances[evidence] > distance:
            ped_distances[evidence] = distance

    return ped_distances
