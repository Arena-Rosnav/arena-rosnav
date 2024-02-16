import os
from typing import List, Optional

from rospkg import RosPack
from task_generator.constants import Config, Constants
from task_generator.shared import DynamicObstacle, ModelWrapper, Namespace, Obstacle
from task_generator.tasks.obstacles import TM_Obstacles
from task_generator.tasks.obstacles.utils import ITF_Obstacle
from task_generator.tasks.task_factory import TaskFactory

import dynamic_reconfigure.client
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

def _get_attrib(element: ET.Element, attribute: str, default: Optional[str] = None) -> str:
    val = element.get(attribute)
    if val is not None:
        return str(val)

    sub_elem = element.find(attribute)
    if sub_elem is not None:
        return str(sub_elem.text)

    if default is not None:
        return default

    raise ValueError(f"attribute {attribute} not found in {element}")


@TaskFactory.register_obstacles(Constants.TaskMode.TM_Obstacles.PARAMETRIZED)
class TM_Parametrized(TM_Obstacles):

    _config: _Config

    PATH_XML: Namespace = Namespace(
        os.path.join(
            RosPack().get_path("arena_bringup"),
            "configs",
            "parametrized"
        )
    )

    @classmethod
    def prefix(cls, *args):
        return super().prefix("parametrized", *args)
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        dynamic_reconfigure.client.Client(
            name=self.NODE_CONFIGURATION,
            config_callback=self.reconfigure
        )

    def reconfigure(self, config):

        xml_path = self.PATH_XML(config["PARAMETRIZED_file"])

        tree = ET.parse(xml_path)
        root = tree.getroot()

        assert isinstance(root, ET.Element) and root.tag == "random", "not a random.xml desc"

        def xml_to_config(config):
            return _ObstacleConfig(
                min=int(_get_attrib(config, "min")),
                max=int(_get_attrib(config, "max")),
                type=_get_attrib(config, "type", ""),
                model=self._PROPS.model_loader.bind(_get_attrib(config, "model"))
            )

        self._config = _Config(
            STATIC = list(map(xml_to_config, root.findall("./static/obstacle") or [])),
            INTERACTIVE = list(map(xml_to_config, root.findall("./static/interactive") or [])),
            DYNAMIC = list(map(xml_to_config, root.findall("./static/dynamic") or [])),
        )


    def reset(self, **kwargs):

        dynamic_obstacles: List[DynamicObstacle] = list()
        obstacles: List[Obstacle] = list()

        # Create static obstacles
        for config in self._config.STATIC:
            for i in range(
                Config.General.RNG.integers(
                    config.min,
                    config.max,
                    endpoint=True
                )
            ):
                obstacle = ITF_Obstacle.create_obstacle(
                    self._PROPS,
                    name=f'S_{config.model.name}_{i+1}',
                    model=config.model
                )
                obstacle.extra["type"] = config.type
                obstacles.append(obstacle)

        # Create interactive obstacles
        for config in self._config.INTERACTIVE:
            for i in range(
                Config.General.RNG.integers(
                    config.min,
                    config.max,
                    endpoint=True
                )
            ):
                obstacle = ITF_Obstacle.create_obstacle(
                    self._PROPS,
                    name=f'S_{config.model.name}_{i+1}',
                    model=config.model
                )
                obstacle.extra["type"] = config.type
                obstacles.append(obstacle)

        # Create dynamic obstacles
        for config in self._config.DYNAMIC:
            for i in range(
                Config.General.RNG.integers(
                    config.min,
                    config.max,
                    endpoint=True
                )
            ):
                obstacle = ITF_Obstacle.create_dynamic_obstacle(
                    self._PROPS,
                    name=f'S_{config.model.name}_{i+1}',
                    model=config.model
                )
                obstacle.extra["type"] = config.type
                dynamic_obstacles.append(obstacle)

        return obstacles, dynamic_obstacles