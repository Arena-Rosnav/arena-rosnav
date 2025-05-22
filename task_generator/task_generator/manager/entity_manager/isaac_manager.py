import typing

from arena_rclpy_mixins.shared import Namespace
from isaacsim_msgs.msg import Person
from isaacsim_msgs.srv import Pedestrian
from task_generator.manager.entity_manager.dummy_manager import \
    DummyEntityManager
from task_generator.manager.entity_manager.utils import ObstacleLayer
from task_generator.shared import DynamicObstacle
from task_generator.simulators import BaseSimulator
from task_generator.simulators.isaac_simulator import IsaacSimulator


class IsaacEntityManager(DummyEntityManager):

    def __init__(self, namespace: Namespace, simulator: BaseSimulator):
        if not isinstance(simulator, IsaacSimulator):
            raise ValueError("IsaacEntityManager only works with IsaacSimulator")
        super().__init__(namespace, simulator)
        self._simulator = typing.cast(IsaacSimulator, self._simulator)

    def spawn_dynamic_obstacles(
        self,
        obstacles: typing.Collection[DynamicObstacle],
    ):
        self._logger.debug(f'spawning {len(obstacles)} dynamic obstacles')
        for obstacle in obstacles:
            self._logger.info(f"Attempting to spawn model: {obstacle.name}")
            self._logger.info(f"waypoints:{obstacle.waypoints}")
            known = self._known_obstacles.get(obstacle.name)
            if known is None:
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle
                )
                model_name = ''  # obstacle.model.name

                self._simulator.services.import_pedestrians.client.call(
                    Pedestrian.Request(
                        people=[Person(
                            stage_prefix=obstacle.name,
                            character_name=model_name,
                            initial_pose=[obstacle.position.x, obstacle.position.y, .1],
                            goal_pose=[obstacle.waypoints[-1].x, obstacle.waypoints[-1].y, 0.0],
                            orientation=obstacle.position.orientation,
                            controller_stats=False,
                            velocity=0.407306046952996
                        )
                        ]
                    )
                )
            else:
                self._simulator.move_entity(obstacle.name, obstacle.position)
            known.layer = ObstacleLayer.INUSE
