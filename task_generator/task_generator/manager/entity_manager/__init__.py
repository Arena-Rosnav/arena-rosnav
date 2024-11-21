import typing
import rclpy
import rclpy.publisher
import rclpy.node
from geometry_msgs.msg import PoseStamped

from task_generator import NodeInterface
from task_generator.manager.entity_manager.utils import ObstacleLayer
from task_generator.manager.utils import WorldMap, WorldWalls
from task_generator.shared import DynamicObstacle, Namespace, Obstacle, PositionOrientation, Robot
from task_generator.simulators import BaseSimulator
from typing import Collection

from task_generator.constants import Constants
from task_generator.utils.registry import Registry


class EntityManager(NodeInterface):

    _goal_pub: rclpy.publisher.Publisher

    def __init__(self, namespace: Namespace,
                 simulator: BaseSimulator, node: rclpy.node.Node = None):
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

        self._goal_pub = self.node.create_publisher(
            PoseStamped,
            self._namespace("/goal"),
            1
        )
        # self._robot_name = self.node.get_parameter('robot_model').value

    def spawn_obstacles(self, obstacles: Collection[Obstacle]):
        """
        Loads given obstacles into the simulator.
        If the object has an interaction radius of > 0,
        then load it as an interactive obstacle instead of static
        """
        raise NotImplementedError()

    def spawn_dynamic_obstacles(self, obstacles: Collection[DynamicObstacle]):
        """
        Loads given obstacles into the simulator.
        Currently by loading a existing sdf file,
        then reaplacing the static values by dynamic ones
        """
        raise NotImplementedError()

    def spawn_walls(self, walls: WorldWalls, heightmap: WorldMap):
        """
        Adds walls to the simulator.
        """
        raise NotImplementedError()

    def unuse_obstacles(self):
        """
        Prepares obstacles for reuse or removal.
        """
        raise NotImplementedError()

    def remove_obstacles(self, purge: ObstacleLayer = ObstacleLayer.UNUSED):
        """
        Removes obstacles from simulator.
        @purge: if False, only remove unused obstacles
        """
        raise NotImplementedError()

    def spawn_robot(self, robot: Robot):
        """
        Spawns a robot.
        @robot: Robot description.
        """
        raise NotImplementedError()

    def move_robot(self, name: str, position: PositionOrientation):
        """
        Moves a robot.
        @name: Robot name
        @position: Target position
        """
        raise NotImplementedError()


EntityManagerRegistry = Registry[Constants.EntityManager, EntityManager]()


@EntityManagerRegistry.register(Constants.EntityManager.DUMMY)
def dummy():

    class DummyEntityManager(EntityManager):
        def __init__(self, namespace: Namespace, simulator: BaseSimulator):
            super().__init__(namespace, simulator)
            self.__logger = self.node.get_logger().get_child('dummy_EM')

        def spawn_obstacles(self, obstacles: Collection[Obstacle]):
            self.__logger.debug(f'spawning {len(obstacles)} static obstacles')
            for obstacle in obstacles:
                self._simulator.spawn_entity(obstacle)

        def spawn_dynamic_obstacles(
                self, obstacles: Collection[DynamicObstacle]):
            self.__logger.debug(f'spawning {len(obstacles)} dynamic obstacles')
            for obstacle in obstacles:
                self._simulator.spawn_entity(obstacle)

        def spawn_walls(self, walls: WorldWalls, heightmap: WorldMap):
            self.__logger.debug(f'spawning {len(walls)} walls')

        def unuse_obstacles(self):
            self.__logger.debug(f'unusing obstacles')

        def remove_obstacles(
                self, purge: ObstacleLayer = ObstacleLayer.UNUSED):
            self.__logger.debug(f'removing obstacles (level {purge})')

        def spawn_robot(self, robot: Robot):
            self.__logger.debug(f'spawning robot {robot.name}')
            self._simulator.spawn_entity(robot)

        def move_robot(self, name: str, position: PositionOrientation):
            self.__logger.debug(
                f'moving robot {name} to {repr(position)}')
            self._simulator.move_entity(name, position)

    return DummyEntityManager
