import dataclasses
import enum
from io import StringIO
import os
from typing import Any, Dict, List, Optional, Union
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import rospkg

import yaml
import rospy
from task_generator.constants import Constants, Config
from task_generator.manager.utils import WorldMap, WorldOccupancy

from task_generator.shared import (
    Model,
    ModelType,
    ModelWrapper,
    Namespace,
    Obstacle,
    ObstacleProps,
    PositionOrientation,
)
from task_generator.utils import Utils


class SDFUtil:
    @staticmethod
    def parse(sdf: str) -> ET.ElementTree:
        file = StringIO(sdf)
        xml = ET.parse(file)
        return xml

    @staticmethod
    def serialize(sdf: ET.ElementTree) -> str:
        file = StringIO()
        sdf.write(file, encoding="Unicode", xml_declaration=True)
        return file.getvalue()

    @staticmethod
    def get_model_root(sdf: ET.ElementTree, tag="model") -> Union[ET.Element, None]:
        root = sdf.getroot()
        if root.tag != tag:
            root = root.find(tag)

        return root

    @staticmethod
    def set_name(sdf: ET.ElementTree, name: str, tag="model") -> None:
        root = SDFUtil.get_model_root(sdf, tag)

        # TODO reconsider whether this should fail silently
        if root is not None:
            root.set("name", name)

    SFM_PLUGIN_SELECTOR = r"""plugin[@filename='libPedestrianSFMPlugin.so']"""
    PEDSIM_PLUGIN_SELECTOR = r"""plugin[@filename='libPedsimGazeboActorPlugin.so']"""
    COLLISONS_PLUGIN_SELECTOR = r"""plugin[@filename='libActorCollisionsPlugin.so']"""

    @staticmethod
    def delete_all(sdf: ET.ElementTree, selector: str) -> int:
        hits = 0
        for plugin_parent in sdf.findall(f".//{selector}/.."):
            for plugin in plugin_parent.findall(f"./{selector}"):
                plugin_parent.remove(plugin)
                hits += 1

        return hits


class ObstacleLayer(enum.IntEnum):
    UNUSED = 0  # unused, could be garbage collected
    INUSE = 1  # in use, but can be unused
    WORLD = 2  # intrinsic part of world


@dataclasses.dataclass
class KnownObstacle:
    obstacle: ObstacleProps
    pedsim_spawned: bool = False
    layer: ObstacleLayer = ObstacleLayer.UNUSED


class KnownObstacles:
    """
    Helper interface to store known obstacles
    """

    # store obstacle descs and whether they have been spawned
    _known_obstacles: Dict[str, KnownObstacle]

    def __init__(self):
        self._known_obstacles = dict()

    def forget(self, name: str):
        """
        Delete obstacle.
        @name: name of obstacle
        """
        if name in self._known_obstacles:
            del self._known_obstacles[name]

    def create_or_get(self, name: str, **kwargs) -> KnownObstacle:
        """
        Get an existing obstacle or create it if it doesn't exist. To overwrite an existing obstacle, first remove it using forget().
        @name: name of obstacle
        @kwargs: arguments passed to KnownObstacle constructor
        """
        if name not in self._known_obstacles:
            self._known_obstacles[name] = KnownObstacle(**kwargs)

        return self._known_obstacles[name]

    def get(self, name: str) -> Optional[KnownObstacle]:
        """
        Get an existing obstacle or return None if it doesn't exist.
        @name: name of obstacle
        """
        return self._known_obstacles.get(name, None)

    def keys(self):
        """
        Get internal dict_keys.
        """
        return self._known_obstacles.keys()

    def values(self):
        """
        Get internal dict_values.
        """
        return self._known_obstacles.values()

    def items(self):
        """
        Get internal dict_items.
        """
        return self._known_obstacles.items()

    def clear(self):
        """
        Clear internal dict.
        """
        return self._known_obstacles.clear()

    def __contains__(self, item: str) -> bool:
        return item in self._known_obstacles


class YAMLUtil:
    @staticmethod
    def check_yaml_path(path: str) -> bool:
        return os.path.isfile(path)

    @staticmethod
    def parse_yaml(content: str):
        return yaml.safe_load(content)

    @staticmethod
    def read_yaml(yaml: Union[StringIO, str]) -> Any:
        if isinstance(yaml, StringIO):
            return YAMLUtil.parse_yaml(yaml.read())

        elif isinstance(yaml, str):
            with open(yaml, "r") as file:
                return YAMLUtil.parse_yaml(file.read())

        else:
            raise ValueError(f"can't process yaml descriptor of type {type(yaml)}")

    @staticmethod
    def serialize(obj: Any):
        return yaml.dump(obj)

    PLUGIN_PROPS_TO_EXTEND: Dict[str, List[str]] = {
        "DiffDrive": ["odom_pub", "twist_sub", "ground_truth_pub"],
        "Laser": ["topic"],
    }

    PLUGIN_PROPS_DEFAULT_VAL = {
        "DiffDrive": {"ground_truth_pub": "odometry/ground_truth"}
    }

    @staticmethod
    def update_frame_id(namespace: Namespace, frame_id: str):
        return f"{namespace.robot_ns}/{frame_id}"

    @staticmethod
    def update_plugins(namespace: Namespace, description: Any) -> Any:
        plugins: List[Dict] = description.get("plugins", [])

        if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
            if rospy.get_param("laser/full_range_laser", False):
                plugins.append(Constants.PLUGIN_FULL_RANGE_LASER.copy())

            for plugin in plugins:
                for prop in YAMLUtil.PLUGIN_PROPS_TO_EXTEND.get(plugin["type"], []):
                    default_val = None
                    if prop not in plugin:
                        default_val = YAMLUtil.PLUGIN_PROPS_DEFAULT_VAL[plugin["type"]][
                            prop
                        ]
                    plugin[prop] = namespace(
                        (
                            plugin.get(prop, "")
                            if not default_val
                            else plugin.get(prop, default_val)
                        ),
                    )

                    # if plugin["type"] == "DiffDrive":
                    #     plugin["odom_frame_id"] = YAMLUtil.update_frame_id(
                    #         namespace, "odom"
                    #     )

            return description

        for plugin in plugins:
            for prop in YAMLUtil.PLUGIN_PROPS_TO_EXTEND.get(plugin["type"], []):
                plugin[prop] = os.path.join(namespace, plugin.get(prop, ""))
        return description


tmp_dir = os.path.join(
    rospkg.RosPack().get_path("arena_simulation_setup"), "tmp", "heightmap"
)
os.makedirs(tmp_dir, exist_ok=True)


def walls_to_obstacle(world_map: WorldMap, height: float = 3) -> Obstacle:
    model_name = "__WALLS"
    heightmap = np.logical_not(
        WorldOccupancy.not_full(world_map.occupancy._walls.grid)
    )[::-1, :]

    dtype = np.uint8

    target_size: int = 2 ** np.ceil(np.log2(max(heightmap.shape))) + 1
    pad_y: int = int(np.floor((target_size - heightmap.shape[0]) / 2))
    pad_x: int = int(np.floor((target_size - heightmap.shape[1]) / 2))

    padded_heightmap = np.pad(
        heightmap,
        np.array(
            [
                (pad_y, pad_y + 1 - heightmap.shape[0] % 2),
                (pad_x, pad_x + 1 - heightmap.shape[1] % 2),
            ]
        ),
        mode="constant",
        constant_values=0,
    )

    img_uri = os.path.join(tmp_dir, f"__WALLS.png")
    cv2.imwrite(img_uri, np.iinfo(dtype).max * padded_heightmap)

    z_offset = -0.1

    mesh = f"""
        <heightmap>
            <uri>{img_uri}</uri>
            <size>{padded_heightmap.shape[1] * world_map.resolution} {padded_heightmap.shape[0] * world_map.resolution} {height - z_offset}</size>
            <pos>{heightmap.shape[1] * .5  * world_map.resolution + world_map.origin.x} {heightmap.shape[0] * .5 * world_map.resolution + world_map.origin.y} {z_offset}</pos>
            <blend></blend>
            <use_terrain_paging>false</use_terrain_paging>
        </heightmap>
        """

    # TODO precompute heightmap as own geometry, gazebo heightmap implementation isn't optimal
    # mesh = ""

    sdf_description = f"""
        <?xml version="1.0" ?>
        <sdf version="1.5">
            <static>true</static>
            <model name="{model_name}">
                <link name="body">
                    <visual name="visual">
                        <pose>0 0 0 0 0 0</pose>
                        <geometry>
                            {mesh}
                        </geometry>
                    </visual>
                    <!--<collision name="collision">
                        <pose>0 0 0 0 0 0</pose>
                        <geometry>
                            {mesh}
                        </geometry>
                    </collision>-->
                </link>
            </model>
        </sdf>
        """

    model = ModelWrapper.Constant(
        model_name,
        models={
            # ModelType.YAML: Model(type=ModelType.YAML, name=model_name, description="", path=""),
            ModelType.SDF: Model(
                type=ModelType.SDF,
                name=model_name,
                description=sdf_description,
                path="",
            )
        },
    )

    return Obstacle(
        position=PositionOrientation(0, 0, 0),
        name=model_name,
        model=model,
        extra=dict(),
    )
