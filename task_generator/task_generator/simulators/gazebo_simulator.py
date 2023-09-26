import rospy
import os
import rospkg
import random
import subprocess
import numpy as np
import math
import re
from scipy.spatial.transform import Rotation

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, DeleteModel, SpawnModel, SpawnModelRequest

from pedsim_srvs.srv import SpawnInteractiveObstacles, SpawnInteractiveObstaclesRequest,SpawnObstacle, SpawnObstacleRequest, SpawnPeds, SpawnPed
from pedsim_msgs.msg import InteractiveObstacle, AgentStates, Waypoints, LineObstacle, Ped, LineObstacles

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion

from std_msgs.msg import Empty
from std_srvs.srv import Empty, SetBool, Trigger

from rospkg import RosPack

from task_generator.simulators.simulator_factory import SimulatorFactory
from tf.transformations import quaternion_from_euler
from ..constants import Constants, Pedsim
from .base_simulator import BaseSimulator
from .simulator_factory import SimulatorFactory
from task_generator.utils import Utils
from nav_msgs.srv import GetMap

import xml.etree.ElementTree as ET
from task_generator.manager.pedsim_manager import PedsimManager

T = Constants.WAIT_FOR_SERVICE_TIMEOUT

@SimulatorFactory.register("gazebo")
class GazeboSimulator(BaseSimulator):
    def __init__(self, namespace):
        super().__init__(namespace)
        self._goal_pub = rospy.Publisher(self._ns_prefix("/goal"), PoseStamped, queue_size=1, latch=True)
        self._robot_name = rospy.get_param("robot_model", "")

        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/set_model_state", timeout=20)

        self._spawn_model_srv = rospy.ServiceProxy(
            self._ns_prefix("gazebo", "spawn_urdf_model"), SpawnModel
        )
        self._move_model_srv = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState, persistent=True
        )
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)

        self.map_manager = None

        self.obstacle_names = []

        self.spawned_obstacles = []
        rospack1 = RosPack()
        pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
        default_actor_model_file = pkg_path + "/models/actor_model.sdf"
        # default_actor_model_file = pkg_path + "/models/prius.sdf"

        actor_model_file = rospy.get_param('~actor_model_file', default_actor_model_file)
        file_xml = open(actor_model_file)
        self.xml_string = file_xml.read()
        print("Waiting for gazebo services...")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        rospy.wait_for_service("gazebo/delete_model")
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.remove_model_srv = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        print("service: spawn_sdf_model is available ....")
        if rospy.get_param("pedsim"):
            rospy.set_param("respawn_dynamic", True)
            rospy.set_param("respawn_static", True)
            rospy.set_param("respawn_interactive", True)
            rospy.Subscriber("/pedsim_simulator/simulated_waypoints", Waypoints, self.interactive_actor_poses_callback)
            rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, self.dynamic_actor_poses_callback)

    def interactive_actor_poses_callback(self, actors):
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
                    
                    z = pkg_path + "/models/"+ob_type+".sdf"
                    file_xml = open(z)
                    x = file_xml.read()

                    model_pose =  Pose(Point(x= actor.position.x-direction_x,
                                        y= actor.position.y-direction_y,
                                        z= actor.position.z)
                                        ,
                                    Quaternion(rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3]) )

                    self.spawn_model(actor_name, x, "", model_pose, "world")
                    self.spawned_obstacles.append(actor_name)
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

                    self.spawn_model(actor_name, x, "", model_pose, "world")
                    self.spawned_obstacles.append(actor_name)
                    rospy.set_param("respawn_static", False)

    def dynamic_actor_poses_callback(self, actors):
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
                self.spawn_model(actor_id, self.xml_string, "", model_pose, "world")
                self.spawned_obstacles.append(actor_id)
                rospy.set_param("respawn_dynamic", False)
            
    def before_reset_task(self):
        self.pause()

    def after_reset_task(self):
        self.unpause()

    def remove_all_obstacles(self):
        for ped in self.spawned_obstacles:
            self.remove_model_srv(str(ped))
        self.spawned_obstacles = []
        return

    # ROBOT
    def publish_goal(self, goal):
        goal_msg = PoseStamped()
        goal_msg.header.seq = 0
        goal_msg.header.stamp = rospy.get_rostime()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.y = goal[1]

        goal_msg.pose.orientation.w = 0
        goal_msg.pose.orientation.x = 0
        goal_msg.pose.orientation.y = 0
        goal_msg.pose.orientation.z = 1

        self._goal_pub.publish(goal_msg)

    def move_robot(self, pos, name=None):
        model_state_request = ModelState()
        model_state_request.model_name = name if name else self._robot_name
        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = 0.1
        pose.orientation = Quaternion(
            *quaternion_from_euler(0.0, 0.0, pos[2], axes="sxyz")
        )
        model_state_request.pose = pose
        model_state_request.reference_frame = "world"

        self._move_model_srv(model_state_request)

    def spawn_robot(self, name, robot_name, namespace_appendix=""):
        request = SpawnModelRequest()

        robot_namespace = self._ns_prefix(namespace_appendix)
        robot_description = GazeboSimulator.get_robot_description(
            robot_name, robot_namespace
        )
        rospy.set_param(os.path.join(robot_namespace, "robot_description"), robot_description)
        rospy.set_param(os.path.join(robot_namespace, "tf_prefix"), robot_namespace)
        request.model_name = name
        request.model_xml = robot_description
        request.robot_namespace = robot_namespace
        request.reference_frame = "world"

        self._spawn_model_srv(request)

    def reset_pedsim_agents(self):
        self._reset_peds_srv()
    
    @staticmethod
    def get_robot_description(robot_name, namespace):
        arena_sim_path = rospkg.RosPack().get_path("arena-simulation-setup")

        return subprocess.check_output([
            "rosrun",
            "xacro",
            "xacro",
            os.path.join(arena_sim_path, "robot", robot_name, "urdf", f"{robot_name}.urdf.xacro"),
            f"robot_namespace:={namespace}"
        ]).decode("utf-8")

    def spawn_obstacles(self, obs):
        pass
