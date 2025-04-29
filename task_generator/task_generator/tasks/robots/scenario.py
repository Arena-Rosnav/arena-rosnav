import json
import os
from typing import NamedTuple

from task_generator.shared import PositionOrientation, PositionRadius
from task_generator.tasks.robots import TM_Robots
from task_generator.utils.ros_params import ROSParamT


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


class TM_Scenario(TM_Robots):
    """
    This class represents a scenario for robots in the task generator.
    It inherits from TM_Robots class and Node class.

    Attributes:
        _config (Config): The configuration object for the scenario.
    """

    _config: ROSParamT[list[_RobotGoal]]

    def _parse_scenario(self, scenario_file: str) -> list[_RobotGoal]:

        scenario_path = os.path.join(
            self.node.conf.Arena.get_world_path(),
            "scenarios",
            scenario_file
        )

        with open(scenario_path) as f:
            scenario = json.load(f)

        return [
            _RobotGoal.parse(robot)
            for robot
            in scenario.get("robots", [])
        ]

    def reset(self, **kwargs):
        """
        Resets the scenario.

        Args:
            kwargs: Additional keyword arguments.

        Returns:
            None
        """

        super().reset(**kwargs)

        SCENARIO_ROBOTS = self._config.value

        # check robot manager length
        managed_robots = list(self._PROPS.robot_managers.values())

        scenario_robots_length = len(SCENARIO_ROBOTS)
        setup_robot_length = len(managed_robots)

        if setup_robot_length > scenario_robots_length:
            managed_robots = managed_robots[:scenario_robots_length]
            self._logger.warn(
                "Robot setup contains more robots than the scenario file.", once=True)

        if scenario_robots_length > setup_robot_length:
            SCENARIO_ROBOTS = SCENARIO_ROBOTS[:setup_robot_length]
            self._logger.warn(
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

    def __init__(self, **kwargs):
        TM_Robots.__init__(self, **kwargs)

        self._config = self.node.ROSParam[list[_RobotGoal]](
            self.namespace('file'),
            'default.json',
            parse=self._parse_scenario,
        )
