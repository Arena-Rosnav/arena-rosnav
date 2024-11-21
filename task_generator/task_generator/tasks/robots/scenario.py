import dataclasses
import json
import os
from typing import List, NamedTuple

from ament_index_python.packages import get_package_share_directory
import rclpy
from rcl_interfaces.msg import SetParametersResult
from task_generator.constants import Constants
from task_generator.shared import PositionOrientation, PositionRadius
from task_generator.tasks.robots import TM_Robots


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


class TM_Scenario(TM_Robots):
    """
    This class represents a scenario for robots in the task generator.
    It inherits from TM_Robots class and Node class.

    Attributes:
        _config (Config): The configuration object for the scenario.
    """

    _config: _Config

    @classmethod
    def prefix(cls, *args):
        return super().prefix("scenario", *args)

    def __init__(self, **kwargs):
        TM_Robots.__init__(self, **kwargs)

        self.node.declare_parameter('SCENARIO_file', '')
        self.node.declare_parameter('map_file', '')
        self.node.add_on_set_parameters_callback(self.parameters_callback)

        # Initial configuration
        self.reconfigure(
            {'SCENARIO_file': self.node.get_parameter('SCENARIO_file').value})

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'SCENARIO_file':
                self.reconfigure({'SCENARIO_file': param.value})
        return SetParametersResult(successful=True)

    def reconfigure(self, config):
        """
        Reconfigures the scenario based on the provided configuration.

        Args:
            config: The configuration object.

        Returns:
            None
        """
        map_file = self.node.get_parameter('map_file').value
        scenario_file = config['SCENARIO_file']

        scenario_path = os.path.join(
            get_package_share_directory('simulation-setup'),
            "worlds",
            map_file,
            "scenarios",
            scenario_file
        )

        with open(scenario_path) as f:
            scenario = json.load(f)

        self._config = _Config(
            robots=[_RobotGoal.parse(robot)
                    for robot in scenario.get("robots", [])]
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
            self.node.get_logger().warn(
                "Robot setup contains more robots than the scenario file.", once=True)

        if scenario_robots_length > setup_robot_length:
            SCENARIO_ROBOTS = SCENARIO_ROBOTS[:setup_robot_length]
            self.node.get_logger().warn(
                "Scenario file contains more robots than setup.", once=True)

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
