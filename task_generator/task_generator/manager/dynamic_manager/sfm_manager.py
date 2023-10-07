#GRADUALLY REPLACE COPIED METHODS FROM THIS FILE WITH NEW NON-PEDSIM IMPLEMENTATIONS

import datetime
import time
from typing import Any, Callable, Dict, Iterable, List, Tuple
from task_generator.manager.dynamic_manager.dynamic_manager import DynamicManager
from task_generator.manager.map_manager import MapManager
from task_generator.simulators.base_simulator import BaseSimulator

import os

from task_generator.constants import Constants, Pedsim

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation
import re

from rospkg import RosPack

import io

from std_msgs.msg import Empty
from std_srvs.srv import Empty, SetBool, Trigger

import xml.etree.ElementTree as ET

from task_generator.shared import DynamicObstacle, Obstacle, Model, ModelType, Waypoint
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

        self.simulator = simulator

        self._ns_prefix = lambda *topic: os.path.join(namespace, *topic)
        self._goal_pub = rospy.Publisher(self._ns_prefix("/goal"), PoseStamped, queue_size=1, latch=True)

        self._robot_name = rospy.get_param("robot_model", "")

        rospy.set_param("respawn_dynamic", True)
        rospy.set_param("respawn_static", True)
        rospy.set_param("respawn_interactive", True)

        pkg_path = RosPack().get_path('pedsim_gazebo_plugin')
        default_actor_model_file = os.path.join(pkg_path, "models", "actor2.sdf")
        
        actor_model_file: str = str(rospy.get_param('~actor_model_file', default_actor_model_file))
        with open(actor_model_file) as f:
            self.default_actor_model = Model(type=ModelType.SDF, description=f.read(), name="actor2")

        self.__spawned_obstacles = []
        self.__namespaces = dict()

    def __index_namespace(self, namespace: str) -> NamespaceIndexer:
        if namespace not in self.__namespaces:
            self.__namespaces[namespace] = NamespaceIndexer(namespace)

        return self.__namespaces[namespace]
    
    def spawn_obstacle(self, obstacle: Obstacle):
        name, free = next(self.__index_namespace(obstacle.name))
        obstacle.name = name
        self.simulator.spawn_obstacle(obstacle)
        self.__spawned_obstacles.append((name, free))
    
    def spawn_dynamic_obstacle(self, obstacle: DynamicObstacle):

        rospy.loginfo("Spawning model: actor_id = %s", obstacle.name)

        name, free = next(self.__index_namespace(obstacle.name))

        model_desc = fill_actor(obstacle.model.description, name=name, pose=obstacle.pose, waypoints=obstacle.waypoints)
        

        obstacle.model = Model(
            type=obstacle.model.type,
            name=obstacle.name,
            description=model_desc
        )

        print("pre", time.time_ns())
        self.simulator.spawn_obstacle(obstacle)
        self.__spawned_obstacles.append((name, free))
        print("post", time.time_ns())
        rospy.set_param("respawn_dynamic", False)

    def spawn_line_obstacle(self, name, _from, _to):
        pass;

    def remove_obstacles(self):
        for name, cleanup in self.__spawned_obstacles:
            print(f"removing {name}")
            self.simulator.delete_obstacle(obstacle_id=name)
            cleanup()
    
    def interactive_actor_poses_callback(self, actors):
        return;
        if rospy.get_param("respawn_interactive"):
            for actor in actors.waypoints:
                if "interactive" in actor.name:
                    actor_name = str(actor.name)
                    orientation = float( re.findall(r'\(.*?\)', str(actor.name))[0].replace("(","").replace(")","").replace(",","."))
                    direction_x = float( actor.name[actor.name.index("{")+1: actor.name.index("}")].replace(",","."))
                    direction_y = float( actor.name[actor.name.index("[")+1: actor.name.index("]")].replace(",","."))
                    ob_type =            actor.name[actor.name.index("&")+1: actor.name.index("!")]
                    rot = Rotation.from_euler('xyz', [0, 0, orientation], degrees=False)
                    rot_quat = rot.as_quat()

                    rospy.loginfo("Spawning interactive: actor_id = %s", actor_name)

                    rospack1 = RosPack()
                    pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
                    
                    z = os.path.join(pkg_path, "models", f"{ob_type}.sdf")

                    with open(z) as file_xml:
                        x = file_xml.read()

                    model_pose =  Pose(Point(x= actor.position.x-direction_x,
                                        y= actor.position.y-direction_y,
                                        z= actor.position.z)
                                        ,
                                    Quaternion(rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3]) )

                    self.simulator.spawn_model(actor_name, x, "", model_pose, "world")
                    self.simulator.spawned_obstacles.append(actor_name)
                    rospy.set_param("respawn_interactive", False)
                
        if rospy.get_param("respawn_static"):
            for actor in actors.waypoints:
                if "static" in actor.name:

                    actor_name = str(actor.name)
                    orientation = float( re.findall(r'\(.*?\)', str(actor.name))[0].replace("(","").replace(")","").replace(",","."))
                    direction_x = float( actor.name[actor.name.index("{")+1: actor.name.index("}")].replace(",","."))
                    direction_y = float( actor.name[actor.name.index("[")+1: actor.name.index("]")].replace(",","."))
                    ob_type = actor.name[actor.name.index("&")+1: actor.name.index("!")]
                    
                    rot = Rotation.from_euler('xyz', [0, 0, orientation], degrees=False)
                    rot_quat = rot.as_quat()
                    
                    rospy.loginfo("Spawning static: actor_id = %s", actor_name)

                    rospack1 = RosPack()
                    pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
                    z = pkg_path +  "/models/table.sdf"
                    file_xml = open(z)
                    x = file_xml.read()

                    model_pose =  Pose(Point(x= actor.position.x-direction_x,
                                            y= actor.position.y-direction_y,
                                            z= actor.position.z)
                                            ,
                                    Quaternion(rot_quat[0],
                                                rot_quat[1],
                                                rot_quat[2],
                                                rot_quat[3]) )

                    self.simulator.spawn_model(actor_name, x, "", model_pose, "world")
                    self.simulator.spawned_obstacles.append(actor_name)
                    rospy.set_param("respawn_static", False)

    def dynamic_actor_poses_callback(self, actors):
        return;
        if rospy.get_param("respawn_dynamic"):
            for actor in actors.agent_states:
                actor_id = str(actor.id)
                actor_pose = actor.pose
                rospy.loginfo("Spawning dynamic obstacle: actor_id = %s", actor_id)

                model_pose = Pose(Point(x= actor_pose.position.x,
                                    y= actor_pose.position.y,
                                    z= actor_pose.position.z),
                                Quaternion(actor_pose.orientation.x,
                                            actor_pose.orientation.y,
                                            actor_pose.orientation.z,
                                            actor_pose.orientation.w) )
                self.simulator.spawn_model(actor_id, self.xml_string, "", model_pose, "world")
                self.simulator.spawned_obstacles.append(actor_id)
                rospy.set_param("respawn_dynamic", False)
