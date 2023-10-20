import os
import random
from typing import List, Optional, Union
import rospy

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

        interactive_obstacles: int = 0 # interactive obstacles still to be implemented

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

        xml_path = os.path.join(
            RosPack().get_path("task_generator"),
            "scenarios",
            "random_scenario.xml")

        tree = ET.parse(xml_path)
        root = tree.getroot()

        obstacles = list()
        for type in ["static", "dynamic", "interactive"]:
            rospy.logerr(f"Adding obstacle types: {type}")
            for obst in root.findall(type + "/obstacle"):
                try:
                    rospy.logerr(f"Adding obstacle {obst.find('name').text}")
                    obstacles.append({
                        "name": obst.find("name").text,
                        "yaml": obst.find("yaml").text,
                        "type": type,
                        "num": int(obst.find("num").text) if not obst.find("num") == None and not obst.find("num") == 0 else 
                                random.randint(int(obst.find("min-num").text), int(obst.find("max-num").text)),
                    })
                except:
                    rospy.logwarn(f"The Scenario file had faulty configuration!")
                    continue

        obstacles_array: Union(List[DynamicObstacle], List[Obstacle])

        self._obstacle_manager.spawn_map_obstacles()

        #TODO load this from the main loaders
        obstacle_path: str = os.path.join(
            RosPack().get_path("arena-simulation-setup"), "obstacles")
        dynamic_obstacle_path: str = os.path.join(
            RosPack().get_path("arena-simulation-setup"), "dynamic_obstacles")
        
        # Create obstacles
        for ob_type in obstacles:
            model = ob_type["name"]
            obstacles_array = list()
            obst_path = obstacle_path

            for _ in range(ob_type["num"]):

                if ob_type["type"] == "dynamic":
                    obstacle = self._create_dynamic_obstacle(name=self._obstacle_manager._dynamic_manager._default_actor_model.name, model=self._obstacle_manager._dynamic_manager._default_actor_model)
                    obst_path = dynamic_obstacle_path
                else: # interactive / static
                    obstacle = self._create_obstacle(name=model, model=self._model_loader.bind(model))

                obstacle.extra["type"] = ob_type["name"]
                obstacle.extra["yaml"] = os.path.join(
                    obst_path, ob_type["yaml"])
                obstacles_array.append(obstacle)

            if len(obstacles_array):
                if ob_type["type"] == "dynamic":
                    self._obstacle_manager.spawn_dynamic_obstacles(
                        setups=obstacles_array)
                else: # interactive / static
                    self._obstacle_manager.spawn_obstacles(
                        setups=obstacles_array)
                    

        return False, (0, 0, 0)
