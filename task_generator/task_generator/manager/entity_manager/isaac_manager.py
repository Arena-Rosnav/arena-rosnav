import random
import typing
import time
from arena_rclpy_mixins.shared import Namespace
from isaacsim_msgs.srv import MovePed
from isaacsim_msg.msg import NavPed
from task_generator.manager.entity_manager.dummy_manager import DummyEntityManager
from task_generator.manager.entity_manager.utils import ObstacleLayer
from task_generator.shared import DynamicObstacle
from task_generator.simulators import BaseSimulator
from task_generator.simulators.isaac_simulator import IsaacSimulator
import rclpy


class IsaacEntityManager(DummyEntityManager):

    def __init__(self, namespace: Namespace, simulator: BaseSimulator):
        if not isinstance(simulator, IsaacSimulator):
            raise ValueError("IsaacEntityManager only works with IsaacSimulator")
        super().__init__(namespace, simulator)
        self._simulator = typing.cast(IsaacSimulator, self._simulator)

        self._walls: list[str] = []
        # F*** Hardcode

    def spawn_dynamic_obstacles(
        self,
        obstacles: typing.Collection[DynamicObstacle],
    ):
        self._logger.debug(f"spawning {len(obstacles)} dynamic obstacles")
        for obstacle in obstacles:
            self._logger.info(f"Attempting to spawn model: {obstacle.name}")
            self._logger.info(f"waypoints:{obstacle.waypoints}")
            known = self._known_obstacles.get(obstacle.name)
            if known is None:
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name, obstacle=obstacle
                )

                self._simulator._spawn_pedestrian(obstacle)

            else:
                self._known_obstacles.forget(obstacle.name)
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name, obstacle=obstacle
                )
                self._simulator._spawn_pedestrian(obstacle)

            known.layer = ObstacleLayer.INUSE
