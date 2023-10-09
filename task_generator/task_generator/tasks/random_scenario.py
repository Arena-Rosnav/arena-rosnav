import os
import random
from typing import List, Optional

from rospkg import RosPack

from task_generator.constants import Constants
from task_generator.tasks.task_factory import TaskFactory

from task_generator.tasks.base_task import CreateObstacleTask

from task_generator.shared import DynamicObstacle, ModelWrapper, Obstacle, Waypoint

import xml.etree.ElementTree as ET


@TaskFactory.register(Constants.TaskMode.RANDOM_SCENARIO)
class RandomScenarioTask(CreateObstacleTask):
    """
        The random task spawns static and dynamic
        obstacles on every reset and will create
        a new robot start and goal position for
        each task.
    """

    def reset(
        self, start=None, goal=None,
        static_obstacles=None, dynamic_obstacles=None,
    ):
        return super().reset(
            lambda: self._reset_robot_and_obstacles(
                start=start, goal=goal,
                static_obstacles=static_obstacles,
                dynamic_obstacles=dynamic_obstacles,
            )
        )

    def _reset_robot_and_obstacles(
        self,
        start=None,
        goal=None,
        dynamic_obstacles: Optional[int] = None,
        static_obstacles: Optional[int] = None,
    ):

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

        dynamic_obstacles = random.randint(
            Constants.Random.MIN_DYNAMIC_OBS,
            Constants.Random.MAX_DYNAMIC_OBS
        ) if dynamic_obstacles is None else dynamic_obstacles

        static_obstacles = random.randint(
            Constants.Random.MIN_STATIC_OBS,
            Constants.Random.MAX_STATIC_OBS
        ) if static_obstacles is None else static_obstacles

        xml_path = os.path.join(
            RosPack().get_path("task_generator"),
            "scenarios",
            "random_scenario.xml")

        tree = ET.parse(xml_path)
        root = tree.getroot()
        num_tables = [int(str(root[0][0].text)), root[0][1].text, root[0][2].text]
        num_shelves = [int(str(root[1][0].text)), root[1]
                       [1].text, root[1][2].text]
        num_adults = [int(str(root[2][0].text)), root[2]
                      [1].text, root[2][2].text]
        num_elder = [int(str(root[3][0].text)), root[3]
                     [1].text, root[3][2].text]
        num_child = [int(str(root[4][0].text)), root[4]
                     [1].text, root[4][2].text]

        dynamic_obstacles_array: List[DynamicObstacle]
        static_obstacles_array: List[Obstacle]
        interactive_obstacles_array: List[Obstacle]

        self._obstacle_manager.spawn_map_obstacles()

        obstacle_path: str = os.path.join(
            RosPack().get_path("arena-simulation-setup"), "obstacles")
        dynamic_obstacle_path: str = os.path.join(
            RosPack().get_path("arena-simulation-setup"), "dynamic_obstacles")

        # Create static obstacles
        for ob_type in []:# [num_tables]:

            model = ob_type[1]

            static_obstacles_array = list()
            for i in range(ob_type[0]):
                obstacle = self._create_obstacle(name=model, model=self._model_loader.bind(model))
                obstacle.extra["type"] = ob_type[1]
                obstacle.extra["yaml"] = os.path.join(
                    obstacle_path, ob_type[2])
                static_obstacles_array.append(obstacle)

            if len(static_obstacles_array):
                self._obstacle_manager.spawn_obstacles(
                    setups=static_obstacles_array)

        # Create interactive obstacles
        for ob_type in [num_shelves]:

            model = ob_type[1]

            interactive_obstacles_array = list()
            for i in range(ob_type[0]):
                obstacle = self._create_obstacle(name=model, model=self._model_loader.bind(model))
                obstacle.extra["type"] = ob_type[1]
                obstacle.extra["yaml"] = os.path.join(obstacle_path, ob_type[2])
                interactive_obstacles_array.append(obstacle)

            if len(interactive_obstacles_array):
                self._obstacle_manager.spawn_obstacles(
                    setups=interactive_obstacles_array)

        # Create dynamic obstacles
        for ob_type in [num_adults, num_elder, num_child]:
            dynamic_obstacles_array = list()
            for i in range(ob_type[0]):
                obstacle = self._create_dynamic_obstacle(name=self._obstacle_manager._dynamic_manager._default_actor_model.name, model=self._obstacle_manager._dynamic_manager._default_actor_model)
                obstacle.extra["type"] = ob_type[1]
                obstacle.extra["yaml"] = os.path.join(
                    dynamic_obstacle_path, ob_type[2])
                dynamic_obstacles_array.append(obstacle)

            if len(dynamic_obstacles_array):
                self._obstacle_manager.spawn_dynamic_obstacles(
                    setups=dynamic_obstacles_array)

        return False, (0, 0, 0)
