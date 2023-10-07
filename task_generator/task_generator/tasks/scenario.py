from re import S
from typing import Any

from task_generator.shared import DynamicObstacle, Obstacle, RobotGoal, Scenario, ScenarioObstacles
import rospy
import rospkg
import os
import sys
import json

from task_generator.constants import TaskMode
from task_generator.tasks.base_task import BaseTask
from task_generator.tasks.task_factory import TaskFactory

from task_generator.manager.obstacle_manager import ObstacleManager


@TaskFactory.register(TaskMode.SCENARIO)
class ScenarioTask(BaseTask):

    scenario: Scenario

    def __init__(
        self,
        obstacle_manager: ObstacleManager,
        robot_managers,
        map_manager,
        **kwargs
    ):
        scenario_file_path = rospy.get_param("~scenario_json_path")

        self.scenario = self.read_scenario_file(scenario_file_path)

        super().__init__(obstacle_manager, robot_managers, map_manager, **kwargs)

        self._check_map_paths()

        # self._set_up_robot_managers()

        self.reset_count = 0
        
        self.desired_resets = self.scenario.resets

        if self.desired_resets <= 0 or self.desired_resets == None:
            rospy.loginfo(
                f"Setting resets to default of {TaskMode.Scenario.RESETS_DEFAULT}"
            )
            self.desired_resets = TaskMode.Scenario.RESETS_DEFAULT

        self.reset_scenario()

    def reset(self):
        if self.reset_count >= self.desired_resets:
            return True

        super().reset(
            lambda: self.reset_scenario()
        )

        self.reset_count += 1

        return False

    def reset_scenario(self):
        # print("resetting scenario in scenario task")
        self.obstacle_manager.reset()
        self.obstacle_manager.spawn_map_obstacles(self.scenario.map)
        self.obstacle_manager.spawn_obstacles(self.scenario.obstacles.static)
        self.obstacle_manager.spawn_obstacles(self.scenario.obstacles.interactive)
        self.obstacle_manager.spawn_dynamic_obstacles(self.scenario.obstacles.dynamic)
        self._reset_robots()

    def read_scenario_file(self, scenario_file_path) -> Scenario:
        with open(scenario_file_path, "r") as file:
            scenario_file = json.load(file)

        def create_obstacle(obs: Any) -> Obstacle:
            obstacle = Obstacle.parse(obs)
            obstacle.model = self.model_loader.load(obs["model"])
            return obstacle

        def create_dynamic_obstacle(obs: Any) -> DynamicObstacle:
            obstacle = DynamicObstacle.parse(obs)
            obstacle.model = self.dynamic_model_loader.load(obs["model"])
            return obstacle

        return Scenario(
            obstacles=ScenarioObstacles(
                static=[create_obstacle(obs) for obs in scenario_file["obstacles"]["static"]],
                interactive=[],
                dynamic=[create_dynamic_obstacle(obs) for obs in scenario_file["obstacles"]["dynamic"]]
            ),
            map=scenario_file["map"],
            resets=scenario_file["resets"],
            robots=[RobotGoal(start=robot["start"], goal=robot["goal"]) for robot in scenario_file["robots"]]
        )

    def _check_map_paths(self):
        static_map = rospy.get_param("map_path")
        scenario_map_path = os.path.join(
            rospkg.RosPack().get_path("arena-simulation-setup"), 
            "maps", 
            self.scenario.map,
            "map.yaml"
        )

        if not static_map == scenario_map_path:
            rospy.logerr("Map path of scenario and static map are not the same. Shutting down.")
            rospy.logerr(f"Scenario Map Path {scenario_map_path}")
            rospy.logerr(f"Static Map Path {static_map}")

            rospy.signal_shutdown("Map path of scenario and static map are not the same.")
            sys.exit()

    def _reset_robots(self):
        for index, robot in enumerate(self.scenario.robots):
            if len(self.robot_managers) <= index:
                break

            manager = self.robot_managers[index]

            manager.reset(start_pos=robot.start, goal_pos=robot.goal)

    def _set_up_robot_managers(self):
        self._check_robot_manager_length()

        super()._set_up_robot_managers()

    def _check_robot_manager_length(self):
        scenario_robots_length = len(self.scenario.robots)
        setup_robot_length = len(self.robot_managers)

        if setup_robot_length > scenario_robots_length:
            self.robot_managers = self.robot_managers[:scenario_robots_length]
            rospy.logwarn("Roboto setup contains more robots than the scenario file.")
            return

        if scenario_robots_length > setup_robot_length:
            self.scenario.robots = self.scenario.robots[:setup_robot_length]
            rospy.logwarn("Scenario file contains more robots than setup.")
