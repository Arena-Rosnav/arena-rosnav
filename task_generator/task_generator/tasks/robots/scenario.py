from arena_rclpy_mixins.ROSParamServer import ROSParamT
from arena_simulation_setup.world import World, RobotGoal
from task_generator.shared import PositionRadius
from task_generator.tasks.robots import TM_Robots


class TM_Scenario(TM_Robots):
    """
    This class represents a scenario for robots in the task generator.
    It inherits from TM_Robots class and Node class.

    Attributes:
        _config (Config): The configuration object for the scenario.
    """

    _config: ROSParamT[list[RobotGoal]]

    def _parse_scenario(self, scenario: str) -> list[RobotGoal]:
        return World(self.node._world_manager.world_name).scenario(scenario).load().robots

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

        self._config = self.node.ROSParam[list[RobotGoal]](
            self.namespace('file'),
            'default.json',
            parse=self._parse_scenario,
        )
