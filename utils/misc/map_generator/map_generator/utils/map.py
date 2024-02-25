import os

import numpy as np
import matplotlib.pyplot as plt
import rospy
import yaml

from PIL import Image
from matplotlib.ticker import FixedLocator

from map_generator.constants import EMPTY_MAP_YAML, DYNAMIC_MAP_YAML, MAP_GENERATOR_NS


def make_image(map: np.ndarray, map_name: str, dir_path: str):
    # create PNG file from occupancy map (1:occupied, 0:free) and
    # now = datetime.datetime.now()
    img = Image.fromarray(((map - 1) ** 2 * 255).astype("uint8"))  # monochromatic image
    imgrgb = img.convert("RGB")
    # map_name = "map_{}".format(now.strftime("%Y_%m_%d_%H_%M_%S")) # create mapname from current datetime

    os.makedirs(os.path.join(dir_path, map_name, "map"), exist_ok=True)

    imgrgb.save(
        os.path.join(dir_path, map_name, "map", f"{map_name}.png")
    )  # save map in map directory

    create_yaml_files(map_name, dir_path)  # create corresponding yaml files
    # create empty map with same size as map
    # create_empty_map(map.shape[0], map.shape[1], map_name, dir_path)
    # make_image_for_human(map,map_name) # create human friendly map png


def make_image_for_human(
    map, map_name
):  # create human-aestethic PNG file from occupancy map (1:occupied, 0:free)
    height = map.shape[0]
    width = map.shape[1]

    fig, ax = plt.subplots(figsize=(5, 5))
    ax.imshow(
        map, cmap="binary", interpolation="none"
    )  # heatmap, interpolation "none" image scaling big->small, "nearest" small->big
    plt.axis("on")

    ax.axes.get_xaxis().set_visible(True)
    ax.axes.get_yaxis().set_visible(True)
    ax.set_xticks(range(0, width, 10))
    ax.set_xticklabels(range(0, width, 10))
    ax.set_yticks(range(0, height, 10))
    ax.set_yticklabels(range(0, height, 10))
    ax.xaxis.set_minor_locator(FixedLocator(range(width)))
    ax.yaxis.set_minor_locator(FixedLocator(range(height)))

    plt.grid("on", which="both", color="black")
    plt.savefig(f"{map_name}/{map_name}_human.png", bbox_inches="tight")


def create_yaml_files(map_name: str, dir_path: str):
    empty_yaml = EMPTY_MAP_YAML
    map_yaml = DYNAMIC_MAP_YAML

    map_res = rospy.get_param(MAP_GENERATOR_NS("map_properties/resolution"))
    empty_yaml["resolution"] = map_res
    map_yaml["resolution"] = map_res

    with open(f"{dir_path}/{map_name}/map/map.yaml", "w") as outfile:
        yaml.dump(map_yaml, outfile, sort_keys=False, default_flow_style=None)
    # with open(f"{dir_path}/{map_name}/map/empty.yaml", "w") as outfile:
    #     yaml.dump(empty_yaml, outfile, sort_keys=False, default_flow_style=None)

    world_yaml_properties = {
        "properties": {"velocity_iterations": 10, "position_iterations": 10}
    }
    world_yaml_layers = {
        "layers": [
            {"name": "static", "map": "map.yaml", "color": [0, 1, 0, 1]}  # ,
            # {"name": "map","map": "map.yaml","color": [0, 0, 1, 1]}
        ]
    }

    with open(f"{dir_path}/{map_name}/map/map.world.yaml", "w") as outfile:
        yaml.dump(
            world_yaml_properties, outfile, sort_keys=False, default_flow_style=False
        )  # somehow the first part must be with default_flow_style=False
        yaml.dump(
            world_yaml_layers, outfile, sort_keys=False, default_flow_style=None
        )  # 2nd part must be with default_flow_style=None


def create_empty_map(height: int, width: int, map_name: str, dir_path: str):
    _map = np.tile(1, [height, width])
    _map[slice(1, height - 1), slice(1, width - 1)] = 0
    img = Image.fromarray(
        ((_map - 1) ** 2 * 255).astype("uint8")
    )  # monochromatic image
    imgrgb = img.convert("RGB")
    os.makedirs(os.path.join(dir_path, map_name, "map"), exist_ok=True)
    imgrgb.save(os.path.join(dir_path, map_name, "map", f"{map_name}.png"))
