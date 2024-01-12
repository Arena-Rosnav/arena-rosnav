import os
import random
from typing import List, Optional

from rospkg import RosPack
from task_generator.constants import Constants
from task_generator.shared import DynamicObstacle, ModelWrapper, Namespace, Obstacle, rosparam_get
from task_generator.tasks.obstacles import TM_Obstacles
from task_generator.tasks.obstacles.utils import ITF_Obstacle
from task_generator.tasks.task_factory import TaskFactory

import dynamic_reconfigure.client
import dataclasses

import xml.etree.ElementTree as ET

@dataclasses.dataclass
class ObstacleConfig:
    min: int
    max: int
    type: str
    model: ModelWrapper

@dataclasses.dataclass
class Config:
    STATIC: List[ObstacleConfig]
    INTERACTIVE: List[ObstacleConfig]
    DYNAMIC: List[ObstacleConfig]

def get_attrib(element: ET.Element, attribute: str, default: Optional[str] = None) -> str:
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

    _config: Config

    PATH_XML: Namespace = Namespace(
        os.path.join(
            RosPack().get_path("arena_bringup"),
            "configs",
            "parametrized"
        )
    )

    @classmethod
    def prefix(cls, *args):
        return super().prefix("scenario")
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        dynamic_reconfigure.client.Client(
            name=self.NODE_CONFIGURATION,
            config_callback=self.reconfigure
        )

    def reconfigure(self, config):

        xml_path = self.PATH_XML(rosparam_get(str, self.NODE_CONFIGURATION("PARAMETRIZED_file")))

        tree = ET.parse(xml_path)
        root = tree.getroot()

        assert isinstance(root, ET.Element) and root.tag == "random", "not a random.xml desc"

        def xml_to_config(config):
            return ObstacleConfig(
                min=int(get_attrib(config, "min")),
                max=int(get_attrib(config, "max")),
                type=get_attrib(config, "type", ""),
                model=self._PROPS.model_loader.bind(get_attrib(config, "model"))
            )

        self._config = Config(
            STATIC = list(map(xml_to_config, root.findall("./static/obstacle") or [])),
            INTERACTIVE = list(map(xml_to_config, root.findall("./static/interactive") or [])),
            DYNAMIC = list(map(xml_to_config, root.findall("./static/dynamic") or []))
        )


    def reset(self, **kwargs):

        dynamic_obstacles: List[DynamicObstacle] = list()
        obstacles: List[Obstacle] = list()

        # Create static obstacles
        for config in self._config.STATIC:
            for i in range(
                random.randint(
                    config.min,
                    config.max
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
                random.randint(
                    config.min,
                    config.max
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
                random.randint(
                    config.min,
                    config.max
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