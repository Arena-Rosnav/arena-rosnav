import dataclasses
import json
import os
from typing import List, NamedTuple

import rospkg
import rospy
from task_generator.constants import Constants
from task_generator.shared import PositionOrientation, PositionRadius, rosparam_get
from task_generator.tasks.robots import TM_Robots
from task_generator.tasks.task_factory import TaskFactory

import dynamic_reconfigure.client


class _RobotGoal(NamedTuple):
    """
    Represents the start and goal positions for a robot.
    """

    start: PositionOrientation
    goal: PositionOrientation

    @staticmethod
    def parse(obj: dict) -> "_RobotGoal":
        """
        Parses a dictionary object and returns a RobotGoal instance.

        Args:
            obj (dict): The dictionary object containing the start and goal positions.

        Returns:
            RobotGoal: The parsed RobotGoal instance.
        """
        return _RobotGoal(
            start=PositionOrientation(*obj.get("start", [])),
            goal=PositionOrientation(*obj.get("goal", [])),
        )


@dataclasses.dataclass
class _Config:
    robots: List[_RobotGoal]


@TaskFactory.register_robots(Constants.TaskMode.TM_Robots.SCENARIO)
class TM_Scenario(TM_Robots):
    """
    This class represents a scenario for robots in the task generator.
    It inherits from TM_Robots class.

    Attributes:
        _config (Config): The configuration object for the scenario.
    """

    _config: _Config

    @classmethod
    def prefix(cls, *args):
        return super().prefix("scenario", *args)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        dynamic_reconfigure.client.Client(
            name=self.NODE_CONFIGURATION, config_callback=self.reconfigure
        )

    def reconfigure(self, config):
        """
        Reconfigures the scenario based on the provided configuration.

        Args:
            config: The configuration object.

        Returns:
            None
        """

        with open(
            os.path.join(
                rospkg.RosPack().get_path("arena_simulation_setup"),
                "worlds",
                rosparam_get(str, "map_file"),
                "scenarios",
                config["SCENARIO_file"]
            )
        ) as f:
            scenario = json.load(f)

        self._config = _Config(
            robots=[_RobotGoal.parse(robot) for robot in scenario.get("robots", [])]
        )

    def reset(self, **kwargs):
        """
        Resets the scenario.

        Args:
            kwargs: Additional keyword arguments.

        Returns:
            None
        """

        super().reset(**kwargs)

        SCENARIO_ROBOTS = self._config.robots

        # check robot manager length
        managed_robots = self._PROPS.robot_managers

        scenario_robots_length = len(SCENARIO_ROBOTS)
        setup_robot_length = len(managed_robots)

        if setup_robot_length > scenario_robots_length:
            managed_robots = managed_robots[:scenario_robots_length]
            rospy.logwarn_once(
                "Roboto setup contains more robots than the scenario file."
            )

        if scenario_robots_length > setup_robot_length:
            SCENARIO_ROBOTS = SCENARIO_ROBOTS[:setup_robot_length]
            rospy.logwarn_once("Scenario file contains more robots than setup.")

        for robot, config in zip(managed_robots, SCENARIO_ROBOTS):
            robot.reset(start_pos=config.start, goal_pos=config.goal)
            self._PROPS.world_manager.forbid(
                [
                    PositionRadius(
                        x=config.start.x, y=config.start.y, radius=robot.safe_distance
                    ),
                    PositionRadius(
                        x=config.goal.x, y=config.goal.y, radius=robot.safe_distance
                    ),
                ]
            )
