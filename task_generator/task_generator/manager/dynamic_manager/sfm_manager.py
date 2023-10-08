#GRADUALLY REPLACE COPIED METHODS FROM THIS FILE WITH NEW NON-PEDSIM IMPLEMENTATIONS

from typing import Any, Callable, Dict, Iterable, List, Tuple
from task_generator.manager.dynamic_manager.dynamic_manager import DynamicManager
from task_generator.simulators.base_simulator import BaseSimulator

import os

from task_generator.constants import Constants

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from scipy.spatial.transform import Rotation


import io


import xml.etree.ElementTree as ET

from task_generator.shared import Model, Waypoint
from task_generator.utils import NamespaceIndexer

T = Constants.WAIT_FOR_SERVICE_TIMEOUT

def fill_actor(xml_string: str, name: str, pose: Pose, waypoints: Iterable[Waypoint]) -> str:
    
    file = io.StringIO(xml_string)
    xml = ET.parse(file)

    xml_actor = xml.getroot()
    if xml_actor.tag != "actor":
        xml_actor = xml.find("actor")
    assert(xml_actor is not None)
    xml_actor.set("name", name)

    xml_pose = xml_actor.find("pose")
    assert(xml_pose is not None)
    rotRPY = Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_euler("xyz", degrees=False)
    xml_pose.text = f"{pose.position.x} {pose.position.y} {pose.position.z} {rotRPY[0]} {rotRPY[1]} {rotRPY[2]}"

    xml_plugin = xml_actor.find(r"""plugin[@filename='libPedestrianSFMPlugin.so']""")
    assert(xml_plugin is not None)
    xml_plugin.append(ET.fromstring(f"<group><model>{name}</model></group>"))
    xml_plugin.set("name", f"{name}_sfm_plugin")

    file = io.StringIO()
    xml.write(file, encoding="Unicode", xml_declaration=True)
    new_xml_string = file.getvalue().replace("__waypoints__", "".join([f"<waypoint>{x} {y} {theta}</waypoint>" for x, y, theta in waypoints]))

    return new_xml_string

class SFMManager(DynamicManager):

    # COPIED FROM PEDSIM_MANAGER

    __spawned_obstacles: List[Tuple[str, Callable[[], Any]]]
    __namespaces: Dict[str, NamespaceIndexer]
    
    def __init__(self, namespace: str, simulator: BaseSimulator):

        super().__init__(namespace, simulator)

        self._ns_prefix = lambda *topic: os.path.join(namespace, *topic)
        self._goal_pub = rospy.Publisher(self._ns_prefix("/goal"), PoseStamped, queue_size=1, latch=True)

        self._robot_name = rospy.get_param("robot_model", "")

        rospy.set_param("respawn_dynamic", True)
        rospy.set_param("respawn_static", True)
        rospy.set_param("respawn_interactive", True)

        self.__spawned_obstacles = []
        self.__namespaces = dict()

    def __index_namespace(self, namespace: str) -> NamespaceIndexer:
        if namespace not in self.__namespaces:
            self.__namespaces[namespace] = NamespaceIndexer(namespace)

        return self.__namespaces[namespace]
    
    def spawn_obstacles(self, obstacles):

        for obstacle in obstacles:
            name, free = next(self.__index_namespace(obstacle.name))
            obstacle.name = name
            self.simulator.spawn_obstacle(obstacle)
            self.__spawned_obstacles.append((name, free))
    
    def spawn_dynamic_obstacles(self, obstacles):

        for obstacle in obstacles:
            rospy.loginfo("Spawning model: actor_id = %s", obstacle.name)

            name, free = next(self.__index_namespace(obstacle.name))

            model_desc = fill_actor(obstacle.model.description, name=name, pose=obstacle.pose, waypoints=obstacle.waypoints)
            

            obstacle.model = Model(
                type=obstacle.model.type,
                name=obstacle.name,
                description=model_desc
            )

            self.simulator.spawn_obstacle(obstacle)
            self.__spawned_obstacles.append((name, free))

        if len(obstacles):
            rospy.set_param("respawn_dynamic", False)

    def spawn_line_obstacle(self, name, _from, _to):
        #TODO
        pass;

    def remove_obstacles(self):
        for name, cleanup in self.__spawned_obstacles:
            # print(f"removing {name}")
            self.simulator.delete_obstacle(obstacle_id=name)
            cleanup()
    
    def interactive_actor_poses_callback(self, actors):
        return;

    def dynamic_actor_poses_callback(self, actors):
        return;