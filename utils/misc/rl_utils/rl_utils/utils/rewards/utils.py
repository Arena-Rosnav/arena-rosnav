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
    # Compute the Euclidean distance between each point and the origin
    distances = np.sqrt(
        point_cloud["x"] ** 2 + point_cloud["y"] ** 2 + point_cloud["z"] ** 2
    )

    # Find the minimum distance
    return np.min(distances)
