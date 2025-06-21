import abc
import typing

import rclpy
import rclpy.publisher
from arena_rclpy_mixins.shared import Namespace
from geometry_msgs.msg import PoseStamped

from task_generator import NodeInterface
from task_generator.constants import Constants
from task_generator.manager.entity_manager.utils import (KnownObstacles,
                                                         ObstacleLayer)
from task_generator.shared import DynamicObstacle, Obstacle, Pose, Robot, Wall
from task_generator.simulators import BaseSimulator
from task_generator.utils.registry import Registry


class EntityManager(NodeInterface, abc.ABC):

    _goal_pub: rclpy.publisher.Publisher
    _known_obstacles: KnownObstacles

    def __init__(
        self,
        namespace: Namespace,
        simulator: BaseSimulator,
    ):
        """
        Initialize dynamic obstacle manager.

        Args:
            namespace: global namespace
            simulator: Simulator instance
            node: ROS Node instance (optional)
        """
        self._simulator = simulator
        self._namespace = namespace

        NodeInterface.__init__(self)

        self._known_obstacles = KnownObstacles()

        self._goal_pub = self.node.create_publisher(
            PoseStamped,
            self._namespace("/goal"),
            1
        )

    def spawn_obstacles(
        self,
        obstacles: typing.Collection[Obstacle]
    ):
        """
        Loads given obstacles into the simulator.
        """
        self._logger.debug(f'spawning {len(obstacles)} static obstacles')
        for obstacle in obstacles:
            if (known := self._known_obstacles.get(obstacle.name)) is not None:
                known.obstacle = obstacle
                self._simulator.move_entity(known.obstacle.name, known.obstacle.pose)
                known.layer = ObstacleLayer.INUSE
            else:
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle
                )

            if not known.spawned:
                if (obs := self._spawn_obstacle_impl(obstacle)):
                    known.obstacle = obs
                    known.spawned = True

            if known.layer == ObstacleLayer.UNUSED:
                if self._simulator.spawn_entity(known.obstacle):
                    known.layer = ObstacleLayer.INUSE

    def spawn_dynamic_obstacles(
        self,
        obstacles: typing.Collection[DynamicObstacle]
    ):
        """
        Loads given obstacles into the simulator.
        """
        self._logger.debug(f'spawning {len(obstacles)} dynamic obstacles')
        for obstacle in obstacles:
            if (known := self._known_obstacles.get(obstacle.name)) is not None:
                known.obstacle = obstacle
                self._simulator.move_entity(known.obstacle.name, known.obstacle.pose)
                known.layer = ObstacleLayer.INUSE
            else:
                known = self._known_obstacles.create_or_get(
                    name=obstacle.name,
                    obstacle=obstacle
                )

            if not known.spawned:
                if (obs := self._spawn_dynamic_obstacle_impl(obstacle)):
                    known.obstacle = obs
                    known.spawned = True

            if known.layer == ObstacleLayer.UNUSED:
                if self._simulator.spawn_entity(known.obstacle):
                    known.layer = ObstacleLayer.INUSE

    def spawn_walls(
        self,
        walls: typing.Collection[Wall]
    ):
        """
        Adds walls to the simulator.
        """
        self._logger.debug(f'spawning {len(walls)} walls')
        self._simulator.spawn_walls(list(walls))
        self._spawn_walls_impl(walls)

    def unuse_obstacles(self):
        """
        Prepares obstacles for reuse or removal.
        """
        self._logger.debug('unusing obstacles')
        self._remove_obstacles_impl()
        for obstacle in self._known_obstacles.values():
            obstacle.spawned = False
            if obstacle.layer == ObstacleLayer.INUSE:
                obstacle.layer = ObstacleLayer.UNUSED

    def remove_obstacles(
        self,
        purge: ObstacleLayer = ObstacleLayer.UNUSED
    ):
        """
        Removes obstacles from simulator.
        @purge: remove obstacles down to this layer
        """
        self._logger.debug(f'removing obstacles (level {purge})')
        if purge >= ObstacleLayer.WORLD: self._simulator.remove_walls()
        for obstacle_id, obstacle in list(self._known_obstacles.items()):
            if purge >= obstacle.layer:
                self._logger.info(f'!???')
                self._simulator.delete_entity(name=obstacle_id)
                self._known_obstacles.forget(name=obstacle_id)

    def spawn_robot(
        self,
        robot: Robot,
    ):
        """
        Spawns a robot.
        @robot: Robot description.
        """
        self._logger.debug(f'spawning robot {robot.name}')
        self._simulator.spawn_entity(robot)
        self._spawn_robot_impl(robot)

    def remove_robot(
        self,
        name: str,
    ):
        """
        Removes a robot from the simulation.
        @name: Robot name
        """
        self._logger.debug(f'removing robot {name}')
        self._simulator.delete_entity(name)
        self._remove_robot_impl(name)

    def move_robot(
        self,
        name: str,
        pose: Pose
    ):
        """
        Moves a robot.
        @name: Robot name
        @position: Target position
        """
        self._logger.debug(
            f'moving robot {name} to {repr(pose)}')
        self._simulator.move_entity(name, pose)
        self._move_robot_impl(name, pose)

    # impl

    @abc.abstractmethod
    def _spawn_obstacle_impl(
        self,
        obstacle: Obstacle,
    ) -> Obstacle | None:
        ...

    @abc.abstractmethod
    def _spawn_dynamic_obstacle_impl(
        self,
        obstacle: DynamicObstacle,
    ) -> DynamicObstacle | None:
        ...

    @abc.abstractmethod
    def _remove_obstacles_impl(
        self,
    ) -> bool:
        ...

    @abc.abstractmethod
    def _spawn_walls_impl(
        self,
        walls: typing.Collection[Wall],
    ) -> bool:
        ...

    @abc.abstractmethod
    def _spawn_robot_impl(
        self,
        robot: Robot,
    ) -> bool:
        ...

    @abc.abstractmethod
    def _remove_robot_impl(
        self,
        name: str,
    ) -> bool:
        ...

    @abc.abstractmethod
    def _move_robot_impl(
        self,
        name: str,
        pose: Pose
    ) -> bool:
        ...


EntityManagerRegistry = Registry[Constants.EntityManager, EntityManager]()


@EntityManagerRegistry.register(Constants.EntityManager.DUMMY)
def dummy():
    from .dummy_manager import DummyEntityManager
    return DummyEntityManager


@EntityManagerRegistry.register(Constants.EntityManager.HUNAV)
def lazy_hunavsim():
    from .hunav_manager.hunav_manager import HunavManager
    return HunavManager


@EntityManagerRegistry.register(Constants.EntityManager.ISAAC)
def isaacsim():
    from .isaac_manager import IsaacEntityManager
    return IsaacEntityManager
