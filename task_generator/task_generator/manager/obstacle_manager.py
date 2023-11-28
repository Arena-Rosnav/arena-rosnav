from typing import Any, Callable, Collection, Iterator

import itertools


from task_generator.manager.entity_manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.utils import ObstacleLayer
from task_generator.manager.utils import World
from task_generator.manager.world_manager import WorldManager
from task_generator.shared import DynamicObstacle, Obstacle
from task_generator.simulators.base_simulator import BaseSimulator


class ObstacleManager:

    _world_manager: WorldManager
    _namespace: str
    _entity_manager: EntityManager
    _simulator: BaseSimulator

    id_generator: Iterator[int]

    def __init__(self, namespace, world_manager, simulator: BaseSimulator, entity_manager: EntityManager):
        self._world_manager = world_manager
        self._namespace = namespace
        self._simulator = simulator

        self._entity_manager = entity_manager

        self.id_generator = itertools.count(434)

    def spawn_world_obstacles(self, world: World):
        """
        Loads given obstacles into the simulator,
        the map file is retrieved from launch parameter "map_file"
        """

        self._entity_manager.spawn_walls(
            walls=world.entities.walls, heightmap=world.map)
        self._entity_manager.spawn_obstacles(
            obstacles=world.entities.obstacles)

    def spawn_dynamic_obstacles(self, setups: Collection[DynamicObstacle]):
        """
        Loads given dynamic obstacles into the simulator.
        To-Do: consider merging with spawn_dynamic_obstacles or simplifying by calling it
        """

        self._entity_manager.spawn_dynamic_obstacles(obstacles=setups)

    def spawn_obstacles(self, setups: Collection[Obstacle]):
        """
        Loads given obstacles into the simulator.
        If the object has an interaction radius of > 0,
        then load it as an interactive obstacle instead of static
        To-Do: consider merging with spawn_obstacles or simplifying by calling it
        """

        self._entity_manager.spawn_obstacles(obstacles=setups)

    def respawn(self, callback: Callable[[], Any]):
        """
        Unuse obstacles, (re-)use them in callback, finally remove unused obstacles
        @callback: Function to call between unuse and remove
        """
        self._entity_manager.unuse_obstacles()
        callback()
        self._entity_manager.remove_obstacles(purge=ObstacleLayer.UNUSED)

    def reset(self, purge: ObstacleLayer = ObstacleLayer.INUSE):
        """
        Unuse and remove all obstacles
        """
        self._entity_manager.remove_obstacles(purge=purge)
