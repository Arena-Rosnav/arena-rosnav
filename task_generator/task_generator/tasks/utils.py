from dataclasses import asdict, dataclass
import json
import os
import sys
from typing import Dict, List, Optional

import numpy as np
import yaml

import rospy
import rospkg
import rospy
from task_generator.shared import DynamicObstacle, ModelWrapper, Obstacle, PositionOrientation, Waypoint

from task_generator.manager.map_manager import MapManager
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.utils import ModelLoader

import xml.etree.ElementTree as ET


class ManagerProps:
    _obstacle_manager: ObstacleManager
    _robot_managers: List[RobotManager]
    _map_manager: MapManager

class ModelloaderProps:
    _model_loader: ModelLoader
    _dynamic_model_loader: ModelLoader

# ObstacleInterface

class ObstacleInterface(ManagerProps):
    """
    Helper methods to fill partially initialized obstacles
    """

    def _create_dynamic_obstacle(self, waypoints: Optional[List[Waypoint]] = None, **kwargs) -> DynamicObstacle:
        """
            Create dynamic obstacle from partial params.
            @name: Name of the obstacle
            @model: ModelWrapper of models
            @position: (optional) Starting position
            @waypoints: (optional) List of waypoints
            @extra: (optional) Extra properties to store
        """

        setup = self._create_obstacle(**kwargs)

        if waypoints is None:

            safe_distance = 0.5
            # the first waypoint
            waypoints = [(*setup.position[:2], safe_distance)]
            safe_distance = 0.1  # the other waypoints don't need to avoid robot
            for j in range(1):
                dist = 0
                while dist < 8:
                    [x2, y2, *
                        _] = self._map_manager.get_random_pos_on_map(safe_distance)
                    dist = np.linalg.norm(
                        [waypoints[-1][0] - x2, waypoints[-1][1] - y2])
                    waypoints.append((x2, y2, 1))

        return DynamicObstacle(**{
            **asdict(setup),
            **dict(waypoints=waypoints)
        })

    def _create_obstacle(self, name: str, model: ModelWrapper, position: Optional[PositionOrientation] = None, extra: Optional[Dict] = None, **kwargs) -> Obstacle:
        """ 
        Create non-dynamic obstacle from partial params.
        @name: Name of the obstacle
        @model: ModelWrapper of models
        @position: (optional) Starting position
        @extra: (optional) Extra properties to store
        """

        safe_distance = 0.5

        if position is None:
            point: Waypoint = self._map_manager.get_random_pos_on_map(
                safe_distance)
            position = (point[0], point[1], np.pi * np.random.random())

        if extra is None:
            extra = dict()

        return Obstacle(
            position=position,
            name=name,
            model=model,
            extra=extra,
            **kwargs
        )


# ScenarioInterface

@dataclass
class ScenarioObstacles:
    dynamic: List[DynamicObstacle]
    static: List[Obstacle]
    interactive: List[Obstacle]

@dataclass
class ScenarioMap:
    yaml: object
    xml: ET.ElementTree
    path: str

@dataclass
class RobotGoal:
    start: PositionOrientation
    goal: PositionOrientation

@dataclass
class Scenario:
    obstacles: ScenarioObstacles
    map: ScenarioMap
    resets: int
    robots: List[RobotGoal]

class ScenarioInterface(ManagerProps, ModelloaderProps):
    """
    Helper methods to handle scenarios
    """

    def _read_scenario_file(self, scenario_file_content: str) -> Scenario:
        """
        Parse scenario file content as Scenario object.
        @scenario_file_content: string content of scenario file
        """

        scenario_file = json.loads(scenario_file_content)

        static_obstacles = [Obstacle.parse(obs, model=self._model_loader.bind(
            obs["model"])) for obs in scenario_file["obstacles"]["static"]]
        interactive_obstacles = []
        dynamic_obstacles = [DynamicObstacle.parse(obs, model=self._dynamic_model_loader.bind(
            obs["model"])) for obs in scenario_file["obstacles"]["dynamic"]]


        base_path = rospkg.RosPack().get_path("arena-simulation-setup")

        map_name = scenario_file["map"]

        map_path = os.path.join(
            base_path,
            "maps",
            map_name,
            "map.yaml"
        )

        with open(map_path) as f:
            map_yaml = yaml.load(f, Loader=yaml.FullLoader)

        with open(os.path.join(base_path, "worlds", map_name, "ped_scenarios", f"{map_name}.xml")) as f:
            map_xml = ET.parse(f)

        scenario = Scenario(
            obstacles=ScenarioObstacles(
                static=static_obstacles,
                interactive=interactive_obstacles,
                dynamic=dynamic_obstacles
            ),
            map=ScenarioMap(yaml=map_yaml, path=map_path, xml=map_xml),
            resets=scenario_file["resets"],
            robots=[RobotGoal(start=robot["start"], goal=robot["goal"])
                    for robot in scenario_file["robots"]]
        )

        self._check_scenario(scenario=scenario)

        return scenario
    
    def _check_scenario(self, scenario: Scenario) -> bool:

        #check map path
        static_map: str = str(rospy.get_param("map_path", ""))
        scenario_map_path = scenario.map.path

        if static_map != scenario_map_path:
            rospy.logerr(
                "Map path of scenario and static map are not the same. Shutting down.")
            rospy.logerr(f"Scenario Map Path {scenario_map_path}")
            rospy.logerr(f"Static Map Path {static_map}")

            rospy.signal_shutdown(
                "Map path of scenario and static map are not the same.")
            sys.exit()
            return False


        #check robot manager length
        scenario_robots_length = len(scenario.robots)
        setup_robot_length = len(self._robot_managers)

        if setup_robot_length > scenario_robots_length:
            self._robot_managers = self._robot_managers[:scenario_robots_length]
            rospy.logwarn(
                "Roboto setup contains more robots than the scenario file.")

        if scenario_robots_length > setup_robot_length:
            scenario.robots = scenario.robots[:setup_robot_length]
            rospy.logwarn("Scenario file contains more robots than setup.")

        return True

    def _setup_scenario(self, scenario: Scenario):

        self._obstacle_manager.spawn_map_obstacles(scenario.map.xml)
        self._obstacle_manager.spawn_obstacles(scenario.obstacles.static)
        self._obstacle_manager.spawn_obstacles(
            scenario.obstacles.interactive)
        self._obstacle_manager.spawn_dynamic_obstacles(
            scenario.obstacles.dynamic)
        
        for index, robot in enumerate(scenario.robots):
            if index >= len(self._robot_managers):
                break

            manager = self._robot_managers[index]

            manager.reset(start_pos=robot.start[:2], goal_pos=robot.goal[:2])
            manager.move_robot_to_pos(pos=robot.start[:2])

    