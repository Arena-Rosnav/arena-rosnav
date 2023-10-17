import os
import random
from typing import List, Optional

from rospkg import RosPack
import rospy

from task_generator.constants import Constants
from task_generator.manager.map_manager import MapManager
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.tasks.scenario import ScenarioTask
from task_generator.tasks.task_factory import TaskFactory
from task_generator.tasks.base_task import BaseTask
from task_generator.shared import DynamicObstacle, Obstacle, Waypoint

import xml.etree.ElementTree as ET

from task_generator.tasks.utils import ObstacleInterface, Scenario, ScenarioInterface, ScenarioMap, ScenarioObstacles


@TaskFactory.register(Constants.TaskMode.RANDOM_SCENARIO)
class RandomScenarioTask(ScenarioTask, ScenarioInterface, ObstacleInterface):
    """
        The random task spawns static and dynamic
        obstacles on every reset and will create
        a new robot start and goal position for
        each task.
    """

    @BaseTask.reset_helper(parent=BaseTask)
    def reset(self, **kwargs):

        def callback():
            self._obstacle_manager.reset()
            self._setup_scenario(self._generate_scenario())
            return False
    
        return callback

    def _generate_scenario(
        self,
        static_obstacles: Optional[int] = None,
        dynamic_obstacles: Optional[int] = None
    ) -> Scenario:

        robot_positions: List[Waypoint] = []  # may be needed in the future idk

        interactive_obstacles: int = 0

        for manager in self._robot_managers:

            start_pos = self._map_manager.get_random_pos_on_map(
                manager.safe_distance)
            goal_pos = self._map_manager.get_random_pos_on_map(
                manager.safe_distance, forbidden_zones=[start_pos])

            manager.reset(start_pos=start_pos[:2], goal_pos=goal_pos[:2])

            robot_positions.append(start_pos)
            robot_positions.append(goal_pos)

        self._obstacle_manager.reset()
        self._map_manager.init_forbidden_zones()

        if dynamic_obstacles is None:
            dynamic_obstacles = random.randint(
                Constants.Random.MIN_DYNAMIC_OBS,
                Constants.Random.MAX_DYNAMIC_OBS
            )

        if static_obstacles is None:
            static_obstacles = random.randint(
                Constants.Random.MIN_STATIC_OBS,
                Constants.Random.MAX_STATIC_OBS
            )

        xml_path = os.path.join(
            RosPack().get_path("task_generator"),
            "scenarios",
            "random_scenario.xml")

        tree = ET.parse(xml_path)
        root = tree.getroot()
        num_tables = [int(str(root[0][0].text)), root[0]
                      [1].text, root[0][2].text]
        num_shelves = [int(str(root[1][0].text)), root[1]
                       [1].text, root[1][2].text]
        num_adults = [int(str(root[2][0].text)), root[2]
                      [1].text, root[2][2].text]
        num_elder = [int(str(root[3][0].text)), root[3]
                     [1].text, root[3][2].text]
        num_child = [int(str(root[4][0].text)), root[4]
                     [1].text, root[4][2].text]

        dynamic_obstacles_array: List[DynamicObstacle] = list()
        static_obstacles_array: List[Obstacle] = list()
        interactive_obstacles_array: List[Obstacle] = list()

        #TODO load this from the main loaders
        obstacle_path: str = os.path.join(
            RosPack().get_path("arena-simulation-setup"), "obstacles")
        dynamic_obstacle_path: str = os.path.join(
            RosPack().get_path("arena-simulation-setup"), "dynamic_obstacles")

        # Create static obstacles
        for ob_type in []:  # [num_tables]:
            model = ob_type[1]
            for i in range(ob_type[0]):
                obstacle = self._create_obstacle(
                    name=f"{model}_static_{len(static_obstacles_array)+1}", model=self._model_loader.bind(model))
                obstacle.extra["type"] = ob_type[1]
                obstacle.extra["yaml"] = os.path.join(
                    obstacle_path, ob_type[2])
                static_obstacles_array.append(obstacle)

        # Create interactive obstacles
        for ob_type in [num_shelves]:
            model = ob_type[1]
            for i in range(ob_type[0]):
                obstacle = self._create_obstacle(
                    name=f"{model}_interactive_{len(interactive_obstacles_array)+1}", model=self._model_loader.bind(model))
                obstacle.extra["type"] = ob_type[1]
                obstacle.extra["yaml"] = os.path.join(
                    obstacle_path, ob_type[2].split(os.extsep, 1)[0], ob_type[2])
                interactive_obstacles_array.append(obstacle)

        # Create dynamic obstacles
        for ob_type in [num_adults, num_elder, num_child]:
            model = self._obstacle_manager._dynamic_manager._default_actor_model.name
            for i in range(ob_type[0]):
                obstacle = self._create_dynamic_obstacle(
                    name=f"{model}_dynamic_{len(dynamic_obstacles_array)+1}", model=self._obstacle_manager._dynamic_manager._default_actor_model)
                obstacle.extra["type"] = ob_type[1]
                obstacle.extra["yaml"] = os.path.join(
                    dynamic_obstacle_path, ob_type[2].split(os.extsep, 1)[0], ob_type[2])
                dynamic_obstacles_array.append(obstacle)

        return Scenario(
            obstacles = ScenarioObstacles(
                dynamic=dynamic_obstacles_array,
                static=static_obstacles_array,
                interactive=interactive_obstacles_array
            ),
            map = ScenarioMap(yaml=dict(), xml=ET.ElementTree(ET.Element("dummy")), path=""),
            resets = 0,
            robots = []
        )
    
        