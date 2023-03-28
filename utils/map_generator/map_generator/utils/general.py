# load_config, get_config_path, get_rosnav_configs
import math
import yaml
from pathlib import Path


# translate the inflation radius from meters to cells
def calc_infl_rad_cells(infl_rad: float, pgm_res: float):
    rad_in_cells = infl_rad * (1.0 / pgm_res)
    return math.ceil(rad_in_cells)


def get_config_path() -> Path:
    p = Path(__file__)
    return p.parent.parent.parent / "configs" / "config.yaml"


def load_config(cfg_location: str) -> dict:
    with open(cfg_location, "r", encoding="utf-8") as target:
        config = yaml.load(target, Loader=yaml.FullLoader)
    return config


def get_rosnav_configs(cfg: dict) -> dict:
    rosnav_cfg = cfg["generator_configs"]["rosnav"]
    return (
        rosnav_cfg["indoor"]
        if rosnav_cfg["map_type"] == "indoor"
        else rosnav_cfg["outdoor"]
    )
