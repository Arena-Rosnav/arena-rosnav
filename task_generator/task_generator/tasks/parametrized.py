import os
import random
from typing import List, Optional

from rospkg import RosPack

from task_generator.constants import Constants
from task_generator.tasks.scenario import ScenarioTask
from task_generator.tasks.task_factory import TaskFactory
from task_generator.tasks.base_task import BaseTask
from task_generator.shared import DynamicObstacle, Obstacle, Waypoint

import xml.etree.ElementTree as ET

from task_generator.tasks.utils import ObstacleInterface, RandomInterface, Scenario, ScenarioInterface, ScenarioMap, ScenarioObstacles
from task_generator.utils import rosparam_get

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

@TaskFactory.register(Constants.TaskMode.PARAMETRIZED)
class ParametrizedTask(BaseTask, ScenarioInterface, RandomInterface, ObstacleInterface):
    """
        The random task spawns static and dynamic
        obstacles on every reset and will create
        a new robot start and goal position for
        each task.
    """

    @BaseTask.reset_helper(parent=BaseTask)
    def reset(self, **kwargs):

        def callback():
            self._obstacle_manager.respawn(lambda: ScenarioInterface._setup_scenario(self, self._generate_scenario()))
            
            return False

        return callback

    def _generate_scenario(
        self,
        static_obstacles: Optional[int] = None,
        dynamic_obstacles: Optional[int] = None
    ) -> Scenario:

        robot_positions: List[Waypoint] = []  # may be needed in the future idk

        for manager in self._robot_managers:

            start_pos = self._map_manager.get_random_pos_on_map(
                manager.safe_distance)
            goal_pos = self._map_manager.get_random_pos_on_map(
                manager.safe_distance, forbidden_zones=[start_pos])

            manager.reset(start_pos=start_pos[:2], goal_pos=goal_pos[:2])

            robot_positions.append(start_pos)
            robot_positions.append(goal_pos)

        self._map_manager.init_forbidden_zones()

        obstacle_ranges = RandomInterface._load_obstacle_ranges(self)

        if dynamic_obstacles is None:
            dynamic_obstacles = random.randint(*obstacle_ranges.dynamic)

        if static_obstacles is None:
            static_obstacles = random.randint(*obstacle_ranges.static)

        xml_path = os.path.join(
            RosPack().get_path("arena_bringup"),
            "configs",
            "parametrized",
            rosparam_get(str, "~configuration/task_mode/parametrized/file")
        )

        tree = ET.parse(xml_path)
        root = tree.getroot()

        assert isinstance(root, ET.Element) and root.tag == "random", "not a random.xml desc"

        dynamic_obstacles_array: List[DynamicObstacle] = list()
        static_obstacles_array: List[Obstacle] = list()
        interactive_obstacles_array: List[Obstacle] = list()


        # Create static obstacles
        for config in root.findall("./static/obstacle") or []:
            for i in range(
                random.randint(
                    int(get_attrib(config, "min")),
                    int(get_attrib(config, "max"))
                )
            ):
                obstacle = self._create_obstacle(
                    name=f'{get_attrib(config, "name")}_static_{i+1}',
                    model=self._model_loader.bind(get_attrib(config, "model"))
                )
                obstacle.extra["type"] = get_attrib(config, "type", "")
                static_obstacles_array.append(obstacle)

        # Create interactive obstacles
        for config in root.findall("./interactive/obstacle") or []:
            for i in range(
                random.randint(
                    int(get_attrib(config, "min")),
                    int(get_attrib(config, "max"))
                )
            ):
                obstacle = self._create_obstacle(
                    name=f'{get_attrib(config, "name")}_interactive_{i+1}',
                    model=self._model_loader.bind(get_attrib(config, "model"))
                )
                obstacle.extra["type"] = get_attrib(config, "type", "")
                static_obstacles_array.append(obstacle)

        # Create dynamic obstacles
        for config in root.findall("./dynamic/obstacle") or []:
            for i in range(
                random.randint(
                    int(get_attrib(config, "min")),
                    int(get_attrib(config, "max"))
                )
            ):
                obstacle = self._create_dynamic_obstacle(
                    name=f'{get_attrib(config, "name")}_dynamic_{i+1}',
                    model=self._dynamic_model_loader.bind(get_attrib(config, "model"))
                )
                obstacle.extra["type"] = get_attrib(config, "type", "")
                dynamic_obstacles_array.append(obstacle)

        return Scenario(
            obstacles=ScenarioObstacles(
                dynamic=dynamic_obstacles_array,
                static=static_obstacles_array,
                interactive=interactive_obstacles_array
            ),
            map=ScenarioMap(
                yaml=dict(),
                xml=ET.ElementTree(
                ET.Element("dummy")),
                path=""
            ),
            robots=[]
        )
