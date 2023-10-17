import dataclasses
import io
from typing import Dict, Optional, Union
import xml.etree.ElementTree as ET

from task_generator.shared import ObstacleProps


class SDFUtil:

    @staticmethod
    def parse(sdf: str) -> ET.ElementTree:
        file = io.StringIO(sdf)
        xml = ET.parse(file)
        return xml

    @staticmethod
    def serialize(sdf: ET.ElementTree) -> str:
        file = io.StringIO()
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
    spawned: bool = False


class KnownObstacles:
    """
    Helper interface to store known obstacles
    """

    # store obstacle descs and whether they have been spawned
    _known_obstacles: Dict[str, KnownObstacle]

    def __init__(self):
        self._known_obstacles = dict()

    def forget(self, name: str):
        del self._known_obstacles[name]

    def create(self, name: str, obstacle: ObstacleProps, spawned: bool = False) -> KnownObstacle:
        if name not in self._known_obstacles:
            self._known_obstacles[name] = KnownObstacle(
                obstacle=obstacle, spawned=spawned)

        return self._known_obstacles[name]

    def get(self, name: str) -> Optional[KnownObstacle]:
        return self._known_obstacles.get(name, None)

    def keys(self):
        return self._known_obstacles.keys()

    def values(self):
        return self._known_obstacles.values()

    def items(self):
        return self._known_obstacles.items()

    def clear(self):
        return self._known_obstacles.clear()
