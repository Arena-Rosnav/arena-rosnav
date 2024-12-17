import typing
from task_generator.manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.utils import KnownObstacles, ObstacleLayer
from task_generator.shared import DynamicObstacle, Namespace, Obstacle, PositionOrientation, Robot, Wall
from task_generator.simulators import BaseSimulator


class DummyEntityManager(EntityManager):

    _known_obstacles: KnownObstacles

    def __init__(self, namespace: Namespace, simulator: BaseSimulator):
        super().__init__(namespace, simulator)
        self._known_obstacles = KnownObstacles()
        self.__logger = self.node.get_logger().get_child('dummy_EM')

    def spawn_obstacles(self, obstacles: typing.Collection[Obstacle]):
        self.__logger.debug(f'spawning {len(obstacles)} static obstacles')
        for obstacle in obstacles:
            known = self._known_obstacles.get(obstacle.name)
            if known is None:
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle
                )
            known.layer = ObstacleLayer.INUSE
            self._simulator.spawn_entity(obstacle)

    def spawn_dynamic_obstacles(
            self, obstacles: typing.Collection[DynamicObstacle]):
        self.__logger.debug(f'spawning {len(obstacles)} dynamic obstacles')
        for obstacle in obstacles:
            known = self._known_obstacles.get(obstacle.name)
            if known is None:
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle
                )
                self._simulator.spawn_entity(obstacle)
            else:
                self._simulator.move_entity(obstacle.name, obstacle.position)
            known.layer = ObstacleLayer.INUSE

    def spawn_walls(self, walls: list[Wall]):
        self.__logger.debug(f'spawning {len(walls)} walls')
        self._simulator.spawn_walls(walls)

    def unuse_obstacles(self):
        self.__logger.debug(f'unusing obstacles')
        for obstacle_id, obstacle in self._known_obstacles.items():
            if obstacle.layer == ObstacleLayer.INUSE:
                obstacle.layer = ObstacleLayer.UNUSED

    def remove_obstacles(
            self, purge: ObstacleLayer = ObstacleLayer.UNUSED):
        self.__logger.debug(f'removing obstacles (level {purge})')

        for obstacle_id, obstacle in list(self._known_obstacles.items()):
            if purge >= obstacle.layer:
                self._known_obstacles.forget(name=obstacle_id)
                self._simulator.delete_entity(name=obstacle_id)

    def spawn_robot(self, robot: Robot):
        self.__logger.debug(f'spawning robot {robot.name}')
        self._simulator.spawn_entity(robot)

    def remove_robot(self, name: str):
        self.__logger.debug(f'removing robot {name}')
        self._simulator.delete_entity(name)

    def move_robot(self, name: str, position: PositionOrientation):
        self.__logger.debug(
            f'moving robot {name} to {repr(position)}')
