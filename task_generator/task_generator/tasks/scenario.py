from re import S
import rospy
import rospkg
import os
import json

from task_generator.constants import TaskMode
from task_generator.tasks.base_task import BaseTask
from task_generator.tasks.task_factory import TaskFactory


@TaskFactory.register(TaskMode.SCENARIO)
class ScenarioTask(BaseTask):
    def __init__(
        self,
        obstacles_manager,
        robot_managers,
        map_manager,
        **kwargs
    ):
        scenario_file_path = rospy.get_param("~scenario_json_path")

        self.scenario_file = self.read_scenario_file(scenario_file_path)

        super().__init__(obstacles_manager, robot_managers, map_manager, **kwargs)

        self._check_map_paths()

        # self._set_up_robot_managers()

        self.reset_count = 0
        
        self.desired_resets = self.scenario_file.get("resets")

        if self.desired_resets <= 0 or self.desired_resets == None:
            rospy.loginfo(
                f"Setting resets to default of {TaskMode.Scenario.RESETS_DEFAULT}"
            )
            self.desired_resets = TaskMode.Scenario.RESETS_DEFAULT

        self.obstacles_manager.start_scenario(self.scenario_file)

    def reset(self):
        if self.reset_count >= self.desired_resets:
            return True

        super().reset(
            lambda: self.reset_scenario()
        )

        self.reset_count += 1

        return False

    def reset_scenario(self):
        self.obstacles_manager.reset_scenario(self.scenario_file)

        self._reset_robots()

    def read_scenario_file(self, scenario_file_path):
        with open(scenario_file_path, "r") as file:
            content = json.load(file)

            return content

    def _check_map_paths(self):
        static_map = rospy.get_param("map_path")
        scenario_map_path = os.path.join(
            rospkg.RosPack().get_path("arena-simulation-setup"), 
            "maps", 
            self.scenario_file["map"],
            "map.yaml"
        )

        if not static_map == scenario_map_path:
            rospy.logerr("Map path of scenario and static map are not the same. Shutting down.")
            rospy.logerr(f"Scenario Map Path {scenario_map_path}")
            rospy.logerr(f"Static Map Path {static_map}")

            rospy.signal_shutdown("Map path of scenario and static map are not the same.")

    def _reset_robots(self):
        for index, robot in enumerate(self.scenario_file["robots"]):
            if len(self.robot_managers) <= index:
                break

            manager = self.robot_managers[index]

            manager.reset(start_pos=robot["start"], goal_pos=robot["goal"])

    def _set_up_robot_managers(self):
        self._check_robot_manager_length()

        super()._set_up_robot_managers()

    def _check_robot_manager_length(self):
        scenario_robots_length = len(self.scenario_file["robots"])
        setup_robot_length = len(self.robot_managers)

        if setup_robot_length > scenario_robots_length:
            self.robot_managers = self.robot_managers[:scenario_robots_length]
            rospy.logwarn("Roboto setup contains more robots than the scenario file.")
            return

        if scenario_robots_length > setup_robot_length:
            self.scenario_file["robots"] = self.scenario_file["robots"][:setup_robot_length]
            rospy.logwarn("Scenario file contains more robots than setup.")