import io
import os
import typing

import numpy as np
import matplotlib.pyplot as plt
import nav_msgs.srv
import rospy
import yaml

from PIL import Image
from matplotlib.ticker import FixedLocator

from map_generator.constants import MAP_FOLDER_NAME, ROSNAV_MAP_FOLDER, DYNAMIC_MAP_YAML, MAP_GENERATOR_NS, CellValue

import nav_msgs.msg
import std_msgs.msg

def save_map(map, extras):
    ...

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

    plt.grid(True, which="both", color="black")
    plt.savefig(f"{map_name}/{map_name}_human.png", bbox_inches="tight")

def create_empty_map(height: int, width: int, map_name: str, dir_path: str):
    _map = np.tile(1, [height, width])
    _map[slice(1, height - 1), slice(1, width - 1)] = 0
    img = Image.fromarray(
        ((_map - 1) ** 2 * 255).astype("uint8")
    )  # monochromatic image
    imgrgb = img.convert("RGB")
    os.makedirs(os.path.join(dir_path, map_name, "map"), exist_ok=True)
    imgrgb.save(os.path.join(dir_path, map_name, "map", f"{map_name}.png"))


def fill_extras(grid_map, map_properties, extras):

    if 'map/map.png' not in extras:
        grayscale_array = np.full(grid_map.shape, (CellValue.EMPTY+CellValue.FULL)/2, dtype=np.uint8)
        grayscale_array[grid_map == CellValue.FULL] = 0
        grayscale_array[grid_map == CellValue.EMPTY] = 255
        

        img = Image.fromarray(grayscale_array)  # monochromatic image
        imgrgb = img.convert("RGB")
        # map_name = "map_{}".format(now.strftime("%Y_%m_%d_%H_%M_%S")) # create mapname from current datetime

        with io.BytesIO() as buffer:
            imgrgb.save(buffer, 'png')  # save map in map directory
            buffer.seek(0)
            extras['map/map.png'] = buffer.read()

    if 'map/map.yaml' not in extras:
        map_yaml = DYNAMIC_MAP_YAML
        map_yaml["resolution"] = map_properties['resolution']
        extras['map/map.yaml'] = yaml.dump(map_yaml, sort_keys=False, default_flow_style=None)

    if 'map/map.world.yaml' not in extras:
        world_yaml = {
            "properties": {"velocity_iterations": 10, "position_iterations": 10},
            "layers": [
                {"name": "static", "map": "map.yaml", "color": [0, 1, 0, 1]}  # ,
                # {"name": "map","map": "map.yaml","color": [0, 0, 1, 1]}
            ]
        }
        extras['map/map.world.yaml'] = yaml.dump(world_yaml, sort_keys=False, default_flow_style=None)

def preprocess_map_data(grid_map: np.ndarray) -> np.ndarray:
    """Preprocesses the grid map data.

    Flips the grid map from [height, width] to [width, height] and flattens it for publishing OccupancyGrid.data.

    Args:
        grid_map (np.ndarray): The grid map to be preprocessed.

    Returns:
        np.ndarray: The preprocessed grid map data.
    """
    # flip from [height, width] to [width, height]
    # grid_map = np.flip(grid_map, axis=0)
    # map currently [0,1] 2D np array needs to be flattened for publishing OccupancyGrid.data
    return (grid_map).flatten().astype(np.int8)


PARAM_MAP_PROPERTIES = MAP_GENERATOR_NS("map_properties")

class MapPublisher:

    def __init__(self):
        self._map_pub = rospy.Publisher(
            "/map",
            nav_msgs.msg.OccupancyGrid,
            queue_size=1,
            latch=True
        )

        self._change_map_publisher = rospy.Publisher(
            'restart_map_server',
            std_msgs.msg.Empty,
            queue_size=1
        )

        self._saved_map = nav_msgs.msg.OccupancyGrid()
        

    def _cb_get_map(self, req: nav_msgs.srv.GetMapRequest):
        return nav_msgs.srv.GetMapResponse(
            map = self._saved_map
        )

    __first_run = True

    def publish_map(self, grid_map: np.ndarray, map_properties: typing.Dict, extras: typing.Dict):

        map_properties['width'] = grid_map.shape[1]
        map_properties['height'] = grid_map.shape[0]

        fill_extras(grid_map, map_properties, extras)

        for path, content in extras.items():
            full_path = ROSNAV_MAP_FOLDER / MAP_FOLDER_NAME / path
            os.makedirs(os.path.dirname(full_path), exist_ok=True)
            with open(full_path, 'w' if isinstance(content, str) else 'wb') as f:
                f.write(content)

        yaml_props = yaml.safe_load(extras['map/map.yaml'])

        occupancy_grid = nav_msgs.msg.OccupancyGrid()
        occupancy_grid.data = preprocess_map_data(grid_map)

        occupancy_grid.info.resolution = map_properties['resolution']
        occupancy_grid.info.width = map_properties['width']
        occupancy_grid.info.height = map_properties['height']

        occupancy_grid.info.origin.position.x = yaml_props['origin'][0]
        occupancy_grid.info.origin.position.y = yaml_props['origin'][1]
        occupancy_grid.info.origin.position.z = yaml_props['origin'][2]

        rospy.set_param(PARAM_MAP_PROPERTIES, map_properties)

        self._saved_map = occupancy_grid
        self._map_pub.publish(occupancy_grid)

        if self.__first_run:
            self.__service = rospy.Service(
                'static_map',
                nav_msgs.srv.GetMap,
                self._cb_get_map
            )
            self.__first_run = False