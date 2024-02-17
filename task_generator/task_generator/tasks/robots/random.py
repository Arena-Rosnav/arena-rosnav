import math
from typing import List, Tuple

from task_generator.constants import Config, Constants
from task_generator.shared import PositionOrientation, PositionRadius
from task_generator.tasks.robots import TM_Robots
from task_generator.tasks.task_factory import TaskFactory


@TaskFactory.register_robots(Constants.TaskMode.TM_Robots.RANDOM)
class TM_Random(TM_Robots):
    """
    A class representing a task generator for random robot positions.

    Inherits from TM_Robots class.
    """

    @classmethod
    def prefix(cls, *args):
        return super().prefix("random", *args)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def reset(self, **kwargs):
        """
        Reset the task generator.

        Args:
            **kwargs: Additional keyword arguments.

        Returns:
            None
        """

        super().reset(**kwargs)

        ROBOT_POSITIONS: List[
            Tuple[PositionOrientation, PositionOrientation]
        ] = kwargs.get("ROBOT_POSITIONS", [])
        biggest_robot = max(robot.safe_distance for robot in self._PROPS.robot_managers)

        for robot_start, robot_goal in ROBOT_POSITIONS:
            self._PROPS.world_manager.forbid(
                [
                    PositionRadius(robot_start.x, robot_start.y, biggest_robot),
                    PositionRadius(robot_goal.x, robot_goal.y, biggest_robot),
                ]
            )

        if len(ROBOT_POSITIONS) < len(self._PROPS.robot_managers):
            to_generate = 2*(len(self._PROPS.robot_managers) - len(ROBOT_POSITIONS))

            orientations = 2 * math.pi * Config.General.RNG.random(to_generate)
            positions = self._PROPS.world_manager.get_positions_on_map(
                n= to_generate,
                safe_dist=biggest_robot
            )

            generated_positions = [
                PositionOrientation(
                    position.x, position.y, orientation
                )
                for (orientation, position) in zip(
                    orientations,
                    positions
                )
            ]

            ROBOT_POSITIONS += list(
                zip(generated_positions[::2], generated_positions[1::2])
            )

        for robot, pos in zip(self._PROPS.robot_managers, ROBOT_POSITIONS):
            robot.reset(start_pos=pos[0], goal_pos=pos[1])
