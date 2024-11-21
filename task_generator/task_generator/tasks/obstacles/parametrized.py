import os
from typing import List, Optional

from ament_index_python.packages import get_package_share_directory
from task_generator.constants import Constants
from task_generator.constants.runtime import Configuration
from task_generator.shared import DynamicObstacle, ModelWrapper, Namespace, Obstacle
from task_generator.tasks.obstacles import TM_Obstacles
from task_generator.tasks.obstacles.utils import ITF_Obstacle

import rclpy
from rcl_interfaces.msg import SetParametersResult

import dataclasses
import xml.etree.ElementTree as ET


@dataclasses.dataclass
class _ObstacleConfig:
    min: int
    max: int
    type: str
    model: ModelWrapper


@dataclasses.dataclass
class _Config:
    STATIC: List[_ObstacleConfig]
    INTERACTIVE: List[_ObstacleConfig]
    DYNAMIC: List[_ObstacleConfig]


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

    _config: _Config

    PATH_XML: Namespace = Namespace(
        os.path.join(
            get_package_share_directory("arena_bringup"),
            "configs",
            "parametrized"
        )
    )

    @classmethod
    def prefix(cls, *args):
        return super().prefix("parametrized", *args)

    def __init__(self, **kwargs):
        TM_Obstacles.__init__(self, **kwargs)

        self.node.declare_parameter('PARAMETRIZED_file', '')
        self.node.add_on_set_parameters_callback(self.parameters_callback)

        # Initial configuration
        self.reconfigure(
            {'PARAMETRIZED_file': self.node.get_parameter('PARAMETRIZED_file').value})

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'PARAMETRIZED_file':
                self.reconfigure({'PARAMETRIZED_file': param.value})
        return SetParametersResult(successful=True)

    def reconfigure(self, config):
        xml_path = self.PATH_XML(config["PARAMETRIZED_file"])

        tree = ET.parse(xml_path)
        root = tree.getroot()

        assert isinstance(
            root, ET.Element) and root.tag == "random", "not a random.xml desc"

        def xml_to_config(config):
            return _ObstacleConfig(
                min=int(_get_attrib(config, "min")),
                max=int(_get_attrib(config, "max")),
                type=_get_attrib(config, "type", ""),
                model=self._PROPS.model_loader.bind(
                    _get_attrib(config, "model"))
            )

        self._config = _Config(
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
        for config in self._config.STATIC:
            for i in range(
                self.node.Configuration.General.RNG.value.integers(
                    config.min,
                    config.max,
                    endpoint=True
                )
            ):
                obstacle = ITF_Obstacle.create_obstacle(
                    self._PROPS,
                    name=f'S_{config.model.name}_{i + 1}',
                    model=config.model
                )
                obstacle.extra["type"] = config.type
                obstacles.append(obstacle)

        # Create interactive obstacles
        for config in self._config.INTERACTIVE:
            for i in range(
                self.node.Configuration.General.RNG.value.integers(
                    config.min,
                    config.max,
                    endpoint=True
                )
            ):
                obstacle = ITF_Obstacle.create_obstacle(
                    self._PROPS,
                    name=f'S_{config.model.name}_{i + 1}',
                    model=config.model
                )
                obstacle.extra["type"] = config.type
                obstacles.append(obstacle)

        # Create dynamic obstacles
        for config in self._config.DYNAMIC:
            for i in range(
                self.node.Configuration.General.RNG.value.integers(
                    config.min,
                    config.max,
                    endpoint=True
                )
            ):
                obstacle = ITF_Obstacle.create_dynamic_obstacle(
                    self._PROPS,
                    name=f'S_{config.model.name}_{i + 1}',
                    model=config.model
                )
                obstacle.extra["type"] = config.type
                dynamic_obstacles.append(obstacle)

        return obstacles, dynamic_obstacles
