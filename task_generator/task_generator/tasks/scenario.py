from dataclasses import dataclass
from typing import List
from task_generator.constants import Constants
from task_generator.manager.map_manager import MapManager
from task_generator.manager.robot_manager import RobotManager

import rospy
import rospkg
import os
import sys
import json

from task_generator.shared import DynamicObstacle, Obstacle, PositionOrientation
from task_generator.tasks.base_task import BaseTask
from task_generator.tasks.task_factory import TaskFactory
from task_generator.manager.obstacle_manager import ObstacleManager


@dataclass
class ScenarioObstacles:
    dynamic: List[DynamicObstacle]
    static: List[Obstacle]
    interactive: List[Obstacle]


ScenarioMap = str


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


@TaskFactory.register(Constants.TaskMode.SCENARIO)
class ScenarioTask(BaseTask):

    _scenario: Scenario

    _reset_count: int
    _desired_resets: int

    def __init__(
        self,
        obstacle_manager: ObstacleManager,
        robot_managers: List[RobotManager],
        map_manager: MapManager,
        **kwargs
    ):
        super().__init__(obstacle_manager=obstacle_manager,
                         robot_managers=robot_managers, map_manager=map_manager, **kwargs)

        scenario_file_path = rospy.get_param("~scenario_json_path")
        self._scenario = self._read_scenario_file(scenario_file_path)

        self._check_robot_manager_length()

        self._check_map_paths()

        self._reset_count = 0

        self._desired_resets = self._scenario.resets
        if not self._desired_resets > 0:
            rospy.loginfo(
                f"Setting resets to default of {Constants.Scenario.RESETS_DEFAULT}"
            )
            self._desired_resets = Constants.Scenario.RESETS_DEFAULT

        self._reset_scenario()

    def reset(self, **kwargs):
        if self._reset_count >= self._desired_resets:
            return True

        super().reset(
            lambda: self._reset_scenario()
        )

        self._reset_count += 1

        return False

    def _reset_scenario(self):
        # print("resetting scenario in scenario task")
        self._obstacle_manager.reset()
        self._obstacle_manager.spawn_map_obstacles(self._scenario.map)
        self._obstacle_manager.spawn_obstacles(self._scenario.obstacles.static)
        self._obstacle_manager.spawn_obstacles(
            self._scenario.obstacles.interactive)
        self._obstacle_manager.spawn_dynamic_obstacles(
            self._scenario.obstacles.dynamic)
        self._reset_robots()

    def _read_scenario_file(self, scenario_file_path) -> Scenario:
        with open(scenario_file_path, "r") as file:
            scenario_file = json.load(file)

        static_obstacles = [Obstacle.parse(obs, model=self._model_loader.bind(
            obs["model"])) for obs in scenario_file["obstacles"]["static"]]
        interactive_obstacles = []
        dynamic_obstacles = [DynamicObstacle.parse(obs, model=self._dynamic_model_loader.bind(
            obs["model"])) for obs in scenario_file["obstacles"]["dynamic"]]

        return Scenario(
            obstacles=ScenarioObstacles(
                static=static_obstacles,
                interactive=interactive_obstacles,
                dynamic=dynamic_obstacles
            ),
            map=scenario_file["map"],
            resets=scenario_file["resets"],
            robots=[RobotGoal(start=robot["start"], goal=robot["goal"])
                    for robot in scenario_file["robots"]]
        )

    def _check_map_paths(self):
        static_map = rospy.get_param("map_path")
        scenario_map_path = os.path.join(
            rospkg.RosPack().get_path("arena-simulation-setup"),
            "maps",
            self._scenario.map,
            "map.yaml"
        )

        if static_map != scenario_map_path:
            rospy.logerr(
                "Map path of scenario and static map are not the same. Shutting down.")
            rospy.logerr(f"Scenario Map Path {scenario_map_path}")
            rospy.logerr(f"Static Map Path {static_map}")

            rospy.signal_shutdown(
                "Map path of scenario and static map are not the same.")
            sys.exit()

    def _reset_robots(self):
        for index, robot in enumerate(self._scenario.robots):
            if len(self._robot_managers) <= index:
                break

            manager = self._robot_managers[index]

            manager.reset(start_pos=robot.start[:2], goal_pos=robot.goal[:2])
            manager.move_robot_to_pos(pos=robot.start[:2])

    def _check_robot_manager_length(self):
        scenario_robots_length = len(self._scenario.robots)
        setup_robot_length = len(self._robot_managers)

        if setup_robot_length > scenario_robots_length:
            self._robot_managers = self._robot_managers[:scenario_robots_length]
            rospy.logwarn(
                "Roboto setup contains more robots than the scenario file.")
            return

        if scenario_robots_length > setup_robot_length:
            self._scenario.robots = self._scenario.robots[:setup_robot_length]
            rospy.logwarn("Scenario file contains more robots than setup.")
