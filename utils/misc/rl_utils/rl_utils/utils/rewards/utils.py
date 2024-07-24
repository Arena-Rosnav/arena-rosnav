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

    def __call__(self, reward_function: "RewardFunction", obs_dict, *args, **kwargs):
        """
        Updates the internal state information in the given reward function.

        Args:
            reward_function (RewardFunction): The reward function to update.
            **kwargs: Additional keyword arguments to pass to the internal state update function.
        """
        reward_function.add_internal_state_info(
            self.key,
            self.func(reward_function=reward_function, obs_dict=obs_dict),
        )


from rl_utils.utils.observation_collector import (
    ObservationDict,
    LaserCollector,
    FullRangeLaserCollector,
)


def min_dist_laser(obs_dict: ObservationDict, *args, **kwargs):
    full_range_laser_scan: FullRangeLaserCollector.data_class = obs_dict.get(
        FullRangeLaserCollector.name, None
    )
    laser_scan: LaserCollector.data_class = obs_dict.get(LaserCollector.name, None)

    if laser_scan is None or len(laser_scan) == 0:
        raise ValueError("Neither LaserScan nor PointCloud data was provided!")

    if full_range_laser_scan is not None:
        return full_range_laser_scan.min()

    return laser_scan.min()


def safe_dist_breached(
    reward_function: "RewardFunction", obs_dict, *args, **kwargs
) -> None:
    if reward_function.distinguished_safe_dist:
        return obs_dict["ped_safe_dist"] or obs_dict["obs_safe_dist"]
    else:
        return (
            reward_function.get_internal_state_info("min_dist_laser")
            <= reward_function.safe_dist
        )


from rl_utils.utils.observation_collector import *


def get_ped_type_min_distances(observation_dict):
    ped_distances = {}

    relative_locations = observation_dict.get(PedestrianRelativeLocation.name, None)
    pedestrian_types = observation_dict.get(PedestrianTypeCollector.name, None)

    if relative_locations is None or pedestrian_types is None:
        return ped_distances

    if len(relative_locations) == 0 or len(pedestrian_types.points) == 0:
        return ped_distances

    distances = np.linalg.norm(relative_locations, axis=1)
    types = np.array([int(type_data.evidence) for type_data in pedestrian_types.points])

    # get the unique types
    for _type in np.unique(types):
        ped_distances[_type] = np.min(distances[types == _type])

    return ped_distances
