import os
import xml.etree.ElementTree as ET
from typing import List, Optional

import attrs
from ament_index_python.packages import get_package_share_directory

from task_generator.shared import (DynamicObstacle, ModelWrapper, Namespace,
                                   Obstacle)
from task_generator.tasks.obstacles import TM_Obstacles
from task_generator.tasks.obstacles.utils import ITF_Obstacle
from task_generator.utils.ros_params import ROSParam


@attrs.define()
class _ParsedConfig:
    @attrs.define()
    class ObstacleConfig:
        min: int
        max: int
        type: str
        model: ModelWrapper

    STATIC: List[ObstacleConfig]
    INTERACTIVE: List[ObstacleConfig]
    DYNAMIC: List[ObstacleConfig]


def _get_attrib(element: ET.Element, attribute: str,
                default: Optional[str] = None) -> str:
    val = element.get(attribute)
    if val is not None:
        return str(val)

    sub_elem = element.find(attribute)
    if sub_elem is not None:
        return str(sub_elem.text)

    if default is not None:
        return default

    raise ValueError(f"attribute {attribute} not found in {element}")


class TM_Parametrized(TM_Obstacles):

    _config: ROSParam[_ParsedConfig]

    PATH_XML: Namespace = Namespace(
        os.path.join(
            get_package_share_directory("arena_bringup"),
            "configs",
            "parametrized"
        )
    )

    def _parse_xml(self, path: str) -> _ParsedConfig:
        if path == '':
            return _ParsedConfig([], [], [])

        xml_path = self.PATH_XML(path)

        tree = ET.parse(xml_path)
        root = tree.getroot()

        assert isinstance(
            root, ET.Element) and root.tag == "random", "not a random.xml desc"

        def xml_to_config(config) -> _ParsedConfig.ObstacleConfig:
            return _ParsedConfig.ObstacleConfig(
                min=int(_get_attrib(config, "min")),
                max=int(_get_attrib(config, "max")),
                type=_get_attrib(config, "type", ""),
                model=self._PROPS.model_loader.bind(
                    _get_attrib(config, "model"))
            )

        return _ParsedConfig(
            STATIC=list(map(xml_to_config,
                        root.findall("./static/obstacle") or [])),
            INTERACTIVE=list(map(xml_to_config,
                                 root.findall("./static/interactive") or [])),
            DYNAMIC=list(map(xml_to_config,
                             root.findall("./static/dynamic") or [])),
        )

    def reset(self, **kwargs):
        dynamic_obstacles: List[DynamicObstacle] = list()
        obstacles: List[Obstacle] = list()

        # Create static obstacles
        for config in self._config.value.STATIC:
            for i in range(
                self.node.conf.General.RNG.value.integers(
                    config.min,
                    config.max,
                    endpoint=True
                )
            ):
                obstacle = ITF_Obstacle.create_obstacle(
                    self.node,
                    self._PROPS,
                    name=f'S_{config.model.name}_{i + 1}',
                    model=config.model
                )
                obstacle.extra["type"] = config.type
                obstacles.append(obstacle)

                # Create interactive obstacles
        for config in self._config.value.INTERACTIVE:
            for i in range(
                self.node.conf.General.RNG.value.integers(
                    config.min,
                    config.max,
                    endpoint=True
                )
            ):
                obstacle = ITF_Obstacle.create_obstacle(
                    self.node,
                    self._PROPS,
                    name=f'S_{config.model.name}_{i + 1}',
                    model=config.model
                )
                obstacle.extra["type"] = config.type
                obstacles.append(obstacle)

                # Create dynamic obstacles
        for config in self._config.value.DYNAMIC:
            for i in range(
                self.node.conf.General.RNG.value.integers(
                    config.min,
                    config.max,
                    endpoint=True
                )
            ):
                obstacle = ITF_Obstacle.create_dynamic_obstacle(
                    self.node,
                    self._PROPS,
                    name=f'S_{config.model.name}_{i + 1}',
                    model=config.model
                )
                obstacle.extra["type"] = config.type
                dynamic_obstacles.append(obstacle)

        return obstacles, dynamic_obstacles

    def __init__(self, **kwargs):
        TM_Obstacles.__init__(self, **kwargs)

        self._config = self.node.ROSParam[_ParsedConfig](
            self.namespace('file'),
            '',
            parse=self._parse_xml
        )
