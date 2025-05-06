import typing
from typing import Collection

import rclpy
import rclpy.node
import rclpy.publisher
from arena_rclpy_mixins.shared import Namespace
from geometry_msgs.msg import PoseStamped

from task_generator import NodeInterface
from task_generator.constants import Constants
from task_generator.manager.entity_manager.utils import ObstacleLayer
from task_generator.shared import (DynamicObstacle, Obstacle,
                                   PositionOrientation, Robot, Wall)
from task_generator.simulators import BaseSimulator
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

    def spawn_walls(self, walls: typing.Collection[Wall]):
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

    def remove_robot(self, name: str):
        """
        Removes a robot from the simulation.
        @name: Robot name
        """
        raise NotImplementedError()


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
