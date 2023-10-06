#GRADUALLY REPLACE COPIED METHODS FROM THIS FILE WITH NEW NON-PEDSIM IMPLEMENTATIONS

import datetime
import time
from typing import Iterable
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

from pedsim_srvs.srv import SpawnInteractiveObstacles, SpawnInteractiveObstaclesRequest,SpawnObstacle, SpawnObstacleRequest, SpawnPeds, SpawnPed
from pedsim_msgs.msg import InteractiveObstacle, AgentStates, Waypoints, LineObstacle, Ped, LineObstacles

from rospkg import RosPack

import io

from std_msgs.msg import Empty
from std_srvs.srv import Empty, SetBool, Trigger

import xml.etree.ElementTree as ET

from task_generator.shared import CreatedDynamicObstacle, CreatedObstacle, CreatedStaticObstacle, Model, ModelType, ObstacleDescriptionPose, Waypoint

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

    simulator: BaseSimulator
    map_manager: MapManager

    def spawn_dynamic_obstacle(self, obstacle, name):

        x = obstacle.position.x
        y = obstacle.position.y

        rospy.loginfo("Spawning model: actor_id = %s", obstacle.id)

        model_name = f"{name}_{obstacle.id}"
        model_pose = Pose(position=Point(x=x, y=y, z=0), orientation=Quaternion(x=0, y=0, z=0, w=1))
        model_desc = fill_actor(self.default_actor_model.description, name=model_name, pose=model_pose, waypoints=obstacle.waypoints)

        print("pre", time.time_ns())
        self.simulator.spawn_obstacle(
            ObstacleDescriptionPose(
                name=model_name,
                model=Model(type=self.default_actor_model.type, description=model_desc),
                pose=model_pose
            )
        )
        print("post", time.time_ns())
        rospy.set_param("respawn_dynamic", False)


    # COPIED FROM PEDSIM_MANAGER
    
    def __init__(self, namespace: str, simulator: BaseSimulator):

        self.simulator = simulator

        self._ns_prefix = lambda *topic: os.path.join(namespace, *topic)
        self._goal_pub = rospy.Publisher(self._ns_prefix("/goal"), PoseStamped, queue_size=1, latch=True)

        self._robot_name = rospy.get_param("robot_model", "")

        rospy.set_param("respawn_dynamic", True)
        rospy.set_param("respawn_static", True)
        rospy.set_param("respawn_interactive", True)
        rospy.Subscriber("/pedsim_simulator/simulated_waypoints", Waypoints, self.interactive_actor_poses_callback)
        rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, self.dynamic_actor_poses_callback)

        self.map_manager = None

        pkg_path = RosPack().get_path('pedsim_gazebo_plugin')
        default_actor_model_file = os.path.join(pkg_path, "models", "actor2.sdf")
        
        actor_model_file: str = str(rospy.get_param('~actor_model_file', default_actor_model_file))
        with open(actor_model_file) as f:
            self.default_actor_model = Model(type=ModelType.SDF, description=f.read())
    
    def spawn_obstacle(self, obstacle, name, model, interaction_radius):

        self.simulator.spawn_obstacle(
            ObstacleDescriptionPose(
                name=name,
                model=model,
                pose=Pose(
                    position=obstacle.position,
                    orientation=Quaternion(x=0, y=0, z=0, w=1)
                )
            )
        )
    

    def spawn_map_obstacles(self):
        return [];
        map = rospy.get_param("map_file")
        map_path = os.path.join(
            RosPack().get_path("arena-simulation-setup"), 
            "worlds", 
            map,
            "ped_scenarios",
            f"{map}.xml"
        )
        tree = ET.parse(map_path)
        root = tree.getroot()

        forbidden_zones = []

        add_pedsim_srv=SpawnObstacleRequest()
        for child in root:
            lineObstacle=LineObstacle()
            lineObstacle.start.x,lineObstacle.start.y=float(child.attrib['x1']),float(child.attrib['y1'])
            lineObstacle.end.x,lineObstacle.end.y=float(child.attrib['x2']),float(child.attrib['y2'])
            add_pedsim_srv.staticObstacles.obstacles.append(lineObstacle)
            forbidden_zones.append([lineObstacle.start.x, lineObstacle.start.y, 1])
            forbidden_zones.append([lineObstacle.end.x, lineObstacle.end.y, 1])

        self.__add_obstacle_srv.call(add_pedsim_srv)
        return forbidden_zones

    # SCENARIO INTEGRATION
    def spawn_dynamic_scenario_obstacles(self, peds):
        return;
        srv = SpawnPeds()
        srv.peds = []
        i = 0
        self.agent_topic_str=''  

        new_peds = [["scenario_ped_" + ped["name"], ped["pos"], ped["waypoints"]] for ped in peds]
        self.spawn_dynamic_obstacles(peds=new_peds) 
        
        max_num_try = 1
        i_curr_try = 0
        while i_curr_try < max_num_try:
        # try to call service
            response=self.__respawn_peds_srv.call(srv.peds)

            if not response.success:  # if service not succeeds, do something and redo service
                # rospy.logwarn(
                #     f"spawn human failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
                # rospy.logwarn(response.message)
                i_curr_try += 1
            else:
                break
        self._peds = peds
        # rospy.set_param(f'{self._ns_prefix()}agent_topic_string', self.agent_topic_str)
        rospy.set_param("respawn_dynamic", True)
        return

    def spawn_scenario_obstacles(self, obstacles, interaction_radius=0.0):
        return;
        srv = SpawnInteractiveObstacles()
        srv.InteractiveObstacles = []
        i = 0
        self.agent_topic_str=''   
        while i < len(obstacles) : 
            msg = InteractiveObstacle()
            obstacle = obstacles[i]
            # msg.id = obstacle[0]

            msg.pose = Pose()
            msg.pose.position.x = obstacle["pos"][0]
            msg.pose.position.y = obstacle["pos"][1]
            msg.pose.position.z = 0

            self.agent_topic_str+=f',{self._ns_prefix()}pedsim_static_obstacle_{i}/0' 
            msg.type = "shelf"
            msg.interaction_radius = interaction_radius
            msg.yaml_path = os.path.join(
                RosPack().get_path("arena-simulation-setup"),
                "obstacles", "shelf.yaml"
            )
            srv.InteractiveObstacles.append(msg)
            i = i+1

        max_num_try = 1
        i_curr_try = 0
        print("trying to call service with static obstacles: ")    

        while i_curr_try < max_num_try:
        # try to call service
            response=self.spawn_interactive_obstacles_srv.call(srv.InteractiveObstacles)

            if not response.success:  # if service not succeeds, do something and redo service
                # rospy.logwarn(
                #     f"spawn human failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
                i_curr_try += 1
            else:
                break
        # self._peds.append(peds)
        rospy.set_param(f'{self._ns_prefix()}agent_topic_string', self.agent_topic_str)
        rospy.set_param("respawn_static", True)
        rospy.set_param("respawn_interactive", True)
        return

    def remove_interactive_obstacles(self):
        return;
        if rospy.get_param("pedsim"):
            self.__remove_all_interactive_obstacles_srv.call()
        return
    

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
