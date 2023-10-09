from typing import Collection, Iterator
import rospy
import os
import xml.etree.ElementTree as ET
import rospkg
from task_generator.manager.dynamic_manager.dynamic_manager import DynamicManager
from task_generator.manager.map_manager import MapManager

from task_generator.shared import DynamicObstacle, Obstacle

from geometry_msgs.msg import Point

import itertools

from task_generator.simulators.base_simulator import BaseSimulator

# GRADUALLY MIGRATE ALL METHODS FOR STATIC OBSTACLES FROM DYNAMIC_MANAGERS TO HERE


class ObstacleManager:

    map_manager: MapManager
    namespace: str
    dynamic_manager: DynamicManager

    first_reset: bool

    id_generator: Iterator[int]

    def __init__(self, namespace, map_manager, simulator: BaseSimulator, dynamic_manager: DynamicManager):
        self.map_manager = map_manager
        self.namespace = namespace

        self.first_reset = True
        self.dynamic_manager = dynamic_manager

        self.id_generator = itertools.count(434)

    # TODO replace with already loaded XML
    def spawn_map_obstacles(self, map_path: str = "map_empty"):
        """
        Loads given obstacles into the simulator,
        the map file is retrieved from launch parameter "map_file"
        """
        map = rospy.get_param("map_file")
        map_path = os.path.join(
            rospkg.RosPack().get_path("arena-simulation-setup"),
            "worlds",
            map_path,
            "ped_scenarios",
            f"{map}.xml"
        )
        tree = ET.parse(map_path)
        root = tree.getroot()

        for child in root:

            _from = Point(float(child.attrib['x1']), float(
                child.attrib['y1']), 0)
            _to = Point(float(child.attrib['x2']),
                        float(child.attrib['y2']), 0)

            identifier = str(next(self.id_generator))

            self.dynamic_manager.spawn_line_obstacle(
                name=f"wall{identifier}",
                _from=_from,
                _to=_to
            )

    def spawn_dynamic_obstacles(self, setups: Collection[DynamicObstacle]):
        """
        Loads given dynamic obstacles into the simulator.
        To-Do: consider merging with spawn_dynamic_obstacles or simplifying by calling it
        """

        self.dynamic_manager.spawn_dynamic_obstacles(obstacles=setups)

    def spawn_obstacles(self, setups: Collection[Obstacle]):
        """
        Loads given obstacles into the simulator.
        If the object has an interaction radius of > 0, 
        then load it as an interactive obstacle instead of static
        To-Do: consider merging with spawn_obstacles or simplifying by calling it
        """

        self.dynamic_manager.spawn_obstacles(obstacles=setups)

    def reset(self):

        if self.first_reset:
            self.first_reset = False
        else:
            self.dynamic_manager.remove_obstacles()
