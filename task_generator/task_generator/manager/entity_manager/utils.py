import dataclasses
from io import StringIO
import os
from typing import Any, Dict, List, Optional, Union
import xml.etree.ElementTree as ET

import yaml
import rospy
from task_generator.constants import Constants

from task_generator.shared import Namespace, ObstacleProps
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


@dataclasses.dataclass
class KnownObstacle:
    obstacle: ObstacleProps
    pedsim_spawned: bool = False
    used: bool = False


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

    PLUGIN_PROPS_TO_CHANGE = {
        "DiffDrive": {"odom_frame_id": lambda ns, robot_name: f"odom"}
    }

    PLUGIN_PROPS_TO_EXTEND: Dict[str, List[str]] = {
        "DiffDrive": ["odom_pub", "twist_sub", "ground_truth_pub"],
        "Laser": ["topic"],
    }

    PLUGIN_PROPS_DEFAULT_VAL = {
        "DiffDrive": {"ground_truth_pub": "odometry/ground_truth"}
    }

    @staticmethod
    def update_plugins(namespace: Namespace, description: Any) -> Any:
        plugins: List[Dict] = description.get("plugins", [])

        if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
            if rospy.get_param("laser/full_range_laser", False):
                plugins.append(Constants.PLUGIN_FULL_RANGE_LASER.copy())

            for plugin in plugins:
                for prop in YAMLUtil.PLUGIN_PROPS_TO_EXTEND.get(plugin["type"], []):
                    plugin[prop] = os.path.join(
                        namespace.robot_ns,
                        plugin.get(prop)
                        if prop in plugin
                        else YAMLUtil.PLUGIN_PROPS_DEFAULT_VAL[plugin["type"]][prop],
                    )

                # for prop in YAMLUtil.PLUGIN_PROPS_TO_CHANGE.get(plugin["type"], []):
                #     plugin[prop] = YAMLUtil.PLUGIN_PROPS_TO_CHANGE[plugin["type"]][
                #         prop
                #     ](ns=namespace.simulation_ns, robot_name=namespace.robot_ns)

            return description

        for plugin in plugins:
            for prop in YAMLUtil.PLUGIN_PROPS_TO_EXTEND.get(plugin["type"], []):
                plugin[prop] = os.path.join(namespace, plugin.get(prop, ""))
        return description
