from typing import Any, Callable, Collection, Iterator

import xml.etree.ElementTree as ET

from task_generator.manager.entity_manager.entity_manager import EntityManager
from task_generator.manager.map_manager import MapManager
from task_generator.shared import DynamicObstacle, Obstacle, WallObstacle
from task_generator.simulators.base_simulator import BaseSimulator

from geometry_msgs.msg import Point

import itertools


class ObstacleManager:
    _map_manager: MapManager
    _namespace: str
    _entity_manager: EntityManager
    _simulator: BaseSimulator

    id_generator: Iterator[int]

    def __init__(
        self,
        namespace,
        map_manager,
        simulator: BaseSimulator,
        entity_manager: EntityManager,
    ):
        self._map_manager = map_manager
        self._namespace = namespace
        self._simulator = simulator

        self._entity_manager = entity_manager

        self.id_generator = itertools.count(434)

    def spawn_map_obstacles(self, map: ET.ElementTree):
        """
        Loads given obstacles into the simulator,
        the map file is retrieved from launch parameter "map_file"
        """

        root = map.getroot()

        self._entity_manager.spawn_walls([
                WallObstacle(
                    name=f"wall{next(self.id_generator)}",
                    start=(float(child.attrib["x1"]), float(child.attrib["y1"])),
                    end=(float(child.attrib["x2"]), float(child.attrib["y2"]))
                ) for child in root
            ])
            

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
        self._entity_manager.remove_obstacles(purge=False)

    def reset(self):
        """
        Unuse and remove all obstacles
        """
        self._entity_manager.remove_obstacles(purge=True)
