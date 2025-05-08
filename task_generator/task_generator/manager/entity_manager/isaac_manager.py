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
        super().__init__(namespace, simulator)

        if not isinstance(self._simulator, IsaacSimulator):
            raise RuntimeError('IsaacEntityManager can only be used in combination with IsaacSimulator')

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
                self._simulator.client['spawn_pedestrian_client'].call_async(
                    Pedestrian.Request(
                        people=[Person(
                            stage_prefix=obstacle.name,
                            character_name=obstacle.model.name,
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
