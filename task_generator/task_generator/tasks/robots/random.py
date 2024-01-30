import math
import random
from typing import List, Optional, Tuple

import numpy as np
from task_generator.constants import Constants, TaskConfig
from task_generator.shared import PositionOrientation, PositionRadius, rosparam_get
from task_generator.tasks.robots import TM_Robots
from task_generator.tasks.task_factory import TaskFactory

import dynamic_reconfigure.client
import dataclasses

@dataclasses.dataclass
class Config:
    SEED: Optional[int]

@TaskFactory.register_robots(Constants.TaskMode.TM_Robots.RANDOM)
class TM_Random(TM_Robots):
    """
    A class representing a task generator for random robot positions.

    Inherits from TM_Robots class.
    """
    _config: Config

    @classmethod
    def prefix(cls, *args):
        return super().prefix("random", *args)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        dynamic_reconfigure.client.Client(
            name=self.NODE_CONFIGURATION, config_callback=self.reconfigure
        )

    def reconfigure(self, config):
        self._config = Config(
            SEED=(lambda x: x if x >= 0 else None)(rosparam_get(int, self.NODE_CONFIGURATION("RANDOM_seed"), -1))
        )

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
            generated_positions = [
                PositionOrientation(
                    position.x, position.y, random.random() * 2 * math.pi
                )
                for position in (
                    self._PROPS.world_manager.get_positions_on_map(
                        n=2 * (len(self._PROPS.robot_managers) - len(ROBOT_POSITIONS)),
                        safe_dist=biggest_robot,
                        rng = np.random.default_rng(self._config.SEED)
                    )
                )
            ]

            ROBOT_POSITIONS += list(
                zip(generated_positions[::2], generated_positions[1::2])
            )

        for robot, pos in zip(self._PROPS.robot_managers, ROBOT_POSITIONS):
            robot.reset(start_pos=pos[0], goal_pos=pos[1])
