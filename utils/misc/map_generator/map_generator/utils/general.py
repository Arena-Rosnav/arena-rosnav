import math
import os
from pathlib import Path

import rospkg
import yaml

from ..constants import MAP_FOLDER_NAME, ROSNAV_MAP_FOLDER


# translate the inflation radius from meters to cells
def calc_infl_rad_cells(infl_rad: float, pgm_res: float):
    rad_in_cells = infl_rad * (1.0 / pgm_res)
    return math.ceil(rad_in_cells)


def get_config_path() -> Path:
    p = Path(rospkg.RosPack().get_path("map_generator"))
    return p / "configs" / "config.yaml"


def get_robot_config_path(robot_name: str) -> Path:
    p = Path(rospkg.RosPack().get_path("arena_simulation_setup"))
    return p / "entities" / "robots" / f"{robot_name}" / "model_params.yaml"


def load_config(cfg_location: str) -> dict:
    with open(cfg_location, "r", encoding="utf-8") as target:
        config = yaml.load(target, Loader=yaml.FullLoader)
    return config


def get_rosnav_configs(cfg: dict) -> dict:
    rosnav_cfg = cfg["rosnav"]
    return (
        rosnav_cfg["indoor"]
        if rosnav_cfg["map_type"] == "indoor"
        else rosnav_cfg["outdoor"]
    )


def delete_distance_map():
    # delete the distance map if it exists
    distance_map_path = ROSNAV_MAP_FOLDER / MAP_FOLDER_NAME / "distance_map.png"
    if os.path.exists(distance_map_path):
        os.remove(distance_map_path)


def load_map_generator_config() -> dict:
    cfg_path = get_config_path()
    return load_config(cfg_path)


def load_robot_config(robot_name: str) -> dict:
    cfg_path = get_robot_config_path(robot_name)
    return load_config(cfg_path)
