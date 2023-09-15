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

T = Constants.WAIT_FOR_SERVICE_TIMEOUT


@SimulatorFactory.register("gazebo")
class GazeboSimulator(BaseSimulator):
  def __init__(self, namespace):
    super().__init__(namespace)

    self._goal_pub = rospy.Publisher(self._ns_prefix("/goal"), PoseStamped, queue_size=1, latch=True)

    self._robot_name = rospy.get_param("robot_model", "")

    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    rospy.wait_for_service("/gazebo/set_model_state")
    if rospy.get_param("pedsim"):
        rospy.wait_for_service("/pedsim_simulator/spawn_peds", timeout=T)
        rospy.wait_for_service("/pedsim_simulator/reset_all_peds", timeout=T)
        rospy.wait_for_service("/pedsim_simulator/remove_all_peds", timeout=T)
        rospy.wait_for_service("pedsim_simulator/respawn_peds" , timeout=T)
        rospy.wait_for_service("pedsim_simulator/respawn_interactive_obstacles" , timeout=T)
        rospy.wait_for_service("pedsim_simulator/remove_all_interactive_obstacles" , timeout=T)
        rospy.wait_for_service("pedsim_simulator/add_obstacle", timeout=20)
    rospy.wait_for_service("/gazebo/set_model_state", timeout=20)

    self._spawn_model_srv = rospy.ServiceProxy(
        self._ns_prefix("gazebo", "spawn_urdf_model"), SpawnModel
    )
    self._move_model_srv = rospy.ServiceProxy(
        "/gazebo/set_model_state", SetModelState, persistent=True
    )
    self._spawn_peds_srv = rospy.ServiceProxy(
        "/pedsim_simulator/spawn_peds", SpawnPeds
    )
    self._remove_peds_srv = rospy.ServiceProxy(
        "/pedsim_simulator/remove_all_peds", SetBool
    )
    self._reset_peds_srv = rospy.ServiceProxy(
        "/pedsim_simulator/reset_all_peds", Trigger
    )
    self.__respawn_interactive_obstacles_srv = rospy.ServiceProxy(
      "pedsim_simulator/respawn_interactive_obstacles" ,SpawnInteractiveObstacles, persistent=True)
   
    self.__remove_all_interactive_obstacles_srv = rospy.ServiceProxy(
      "pedsim_simulator/remove_all_interactive_obstacles" ,Trigger)

    self.spawn_interactive_obstacles_srv = rospy.ServiceProxy(
      "pedsim_simulator/spawn_interactive_obstacles" ,SpawnInteractiveObstacles, persistent=True)

    self.__respawn_peds_srv = rospy.ServiceProxy(
        "pedsim_simulator/respawn_peds" , SpawnPeds, persistent=True)

    self._spawn_peds_srv = rospy.ServiceProxy(
        "pedsim_simulator/spawn_peds", SpawnPeds)

    self.__add_obstacle_srv = rospy.ServiceProxy(
        "pedsim_simulator/add_obstacle" ,SpawnObstacle, persistent=True)

    self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
    self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)

    self.map_manager = None


    if rospy.get_param("pedsim"):
      self.spawned_obstacles = []

      rospack1 = RosPack()
      pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
      default_actor_model_file = pkg_path + "/models/actor_model.sdf"
      # default_actor_model_file = pkg_path + "/models/table.sdf"
      # default_actor_model_file = pkg_path + "/models/actor2.sdf"
      # default_actor_model_file = pkg_path + "/models/forklift_robot-master/model.sdf"
      # default_actor_model_file = pkg_path + "/models/test_static_obstacle.sdf"
      # default_actor_model_file = pkg_path + "/models/forklift3.sdf"
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
      rospy.set_param("respawn_dynamic", True)
      rospy.set_param("respawn_static", True)
      rospy.set_param("respawn_interactive", True)
      rospy.Subscriber("/pedsim_simulator/simulated_waypoints", Waypoints, self.interactive_actor_poses_callback)
      rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, self.dynamic_actor_poses_callback)
      # rospy.Subscriber("/pedsim_simulator/simulated_walls", LineObstacles, self.static_actor_poses_callback)

  # def static_actor_poses_callback(self, actors):
  #       if rospy.get_param("respawn_static"):
  #         file_xml = open(self.pkg_path + "/models/actor_model.sdf")
  #         static_xml_string = file_xml.read()
  #         for actor in actors.LineObstacles:
  #           actor_name = str(actor.name)
  #           rospy.loginfo("Spawning static obstacle: actor_id = %s", actor_name)

  #           model_pose =  Pose(Point(x= (actor.start.x+actor.start.y)/2,
  #                                 y= (actor.end.y+actor.end.y)/2,
  #                                 z= 0)
  #                                 ,
  #                           Quaternion(0,
  #                                       0,
  #                                       0,
  #                                       1) )
  #           self.spawn_model(actor_name, static_xml_string, "", model_pose, "world")
  #       rospy.set_param("respawn_static", False)

  def interactive_actor_poses_callback(self, actors):
    if rospy.get_param("respawn_interactive"):
        for actor in actors.waypoints:
          if "interactive" in actor.name:
            # print(actor.name)
            
            actor_name = str(actor.name)
            orientation = float( re.findall(r'\(.*?\)', str(actor.name))[0].replace("(","").replace(")","").replace(",","."))
            direction_x = float( actor.name[actor.name.index("{")+1: actor.name.index("}")].replace(",","."))
            direction_y = float( actor.name[actor.name.index("[")+1: actor.name.index("]")].replace(",","."))
            # print("direction_x",direction_x)
            #   print("direction_y",direction_y)

            # Convert to quaternions and print
            rot = Rotation.from_euler('xyz', [0, 0, orientation], degrees=False)
            rot_quat = rot.as_quat()
            # print(rot_quat)

            rospy.loginfo("Spawning interactive: actor_id = %s", actor_name)
            #   x = open(rospy.get_param('~actor_model_file', RosPack().get_path('pedsim_gazebo_plugin') + "/models/table.sdf")).read

            rospack1 = RosPack()
            pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
            z = pkg_path + "/models/table.sdf"
            file_xml = open(z)
            x = file_xml.read()

            model_pose =  Pose(Point(x= actor.position.x-direction_x,
                                  y= actor.position.y-direction_y,
                                  z= actor.position.z)
                                  ,
                            Quaternion(rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3]) )

            # print("interactive spawning with: ", z)
            self.spawn_model(actor_name, x, "", model_pose, "world")
            self.spawned_obstacles.append(actor_name)
            rospy.set_param("respawn_interactive", False)
            
    if rospy.get_param("respawn_static"):
      for actor in actors.waypoints:
        if "static" in actor.name:
          # print(actor.name)

          actor_name = str(actor.name)
          orientation = float( re.findall(r'\(.*?\)', str(actor.name))[0].replace("(","").replace(")","").replace(",","."))
          direction_x = float( actor.name[actor.name.index("{")+1: actor.name.index("}")].replace(",","."))
          direction_y = float( actor.name[actor.name.index("[")+1: actor.name.index("]")].replace(",","."))

          rot = Rotation.from_euler('xyz', [0, 0, orientation], degrees=False)
          rot_quat = rot.as_quat()
          
          rospy.loginfo("Spawning static: actor_id = %s", actor_name)

          rospack1 = RosPack()
          pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
          z = pkg_path + "/models/bookshelf.sdf"
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

          # print("static spawning with: ", z)
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
            # new_xml_string= self.xml_string.replace("0 0 0.75",str(actor_pose.position.x)+" "+str(actor_pose.position.y) +" 0.75")
            # new_xml_string= new_xml_string.replace("actor1",actor_id)
            # print(new_xml_string)
            self.spawn_model(actor_id, self.xml_string, "", model_pose, "world")
            # self.spawned_obstacles.append(actor_id)
            rospy.set_param("respawn_dynamic", False)
    #   else:
    #     for actor in actors.agent_states:
    #         # print("moving", str(actor.id))
    #         model_state_request = ModelState()
    #         model_state_request.model_name = str(actor.id)
    #         actor_pose = actor.pose
    #         model_state_request.pose = Pose(Point(x= actor_pose.position.x,
    #                               y= actor_pose.position.y,
    #                               z= actor_pose.position.z),
    #                         Quaternion(actor_pose.orientation.x,
    #                                     actor_pose.orientation.y,
    #                                     actor_pose.orientation.z,
    #                                     actor_pose.orientation.w) )
    #         model_state_request.reference_frame = "world"
    #         self._move_model_srv(model_state_request)
            
  def before_reset_task(self):
    self.pause()

  def after_reset_task(self):
    self.unpause()

    # PEDSIM INTEGRATION

  def create_pedsim_static_obstacle(self, i, map_manager, forbidden_zones):
      # self.map_manager = map_manager
      # safe_distance = 0.5
      # [x, y, theta] = self.map_manager.get_random_pos_on_map(safe_distance, forbidden_zones) # check later for the need of free indicies and map papram
      # [a, b, theta] = self.map_manager.get_random_pos_on_map(safe_distance, forbidden_zones) # check later for the need of free indicies and map papram
      # obstacle=[[x, y], [a,b]]
      # return obstacle 
      self.map_manager = map_manager
      safe_distance = 0.5
      [x, y, theta] = self.map_manager.get_random_pos_on_map(safe_distance, forbidden_zones) # check later for the need of free indicies and map papram
      # print(obstacles[i])
      
      ped=np.array([i+1, [x, y, 0.0]],dtype=object)
      return ped

  def create_pedsim_interactive_obstacle(self, i, map_manager, forbidden_zones):
      self.map_manager = map_manager
      safe_distance = 0.5
      [x, y, theta] = self.map_manager.get_random_pos_on_map(safe_distance, forbidden_zones) # check later for the need of free indicies and map papram
      # print(obstacles[i])
      ped=np.array([i+1, [x, y, 0.0]],dtype=object)
      return ped

  def create_pedsim_dynamic_obstacle(self,i, map_manager, forbidden_zones):
      self.map_manager = map_manager
      safe_distance = 0.5
      [x, y, theta] = self.map_manager.get_random_pos_on_map(safe_distance, forbidden_zones) # check later for the need of free indicies and map papram
      waypoints = np.array( [x, y, 1]).reshape(1, 3) # the first waypoint
      safe_distance = 0.1 # the other waypoints don't need to avoid robot
      for j in range(10): # noote was 1000
          dist = 0
          while dist < 8:
              [x2, y2, theta2] = self.map_manager.get_random_pos_on_map( safe_distance, forbidden_zones)
              dist = np.linalg.norm([waypoints[-1,0] - x2,waypoints[-1,1] - y2])
          waypoints = np.vstack([waypoints, [x2, y2, 1]])
      ped=np.array([i+1, [x, y, 0.0], waypoints],dtype=object)
      return ped

  def spawn_pedsim_static_obstacles(self, obstacles):
      srv = SpawnInteractiveObstacles()
      srv.InteractiveObstacles = []
      i = 0
      self.agent_topic_str=''   
      while i < len(obstacles) : 
          msg = InteractiveObstacle()
          obstacle = obstacles[i]
          # msg.id = obstacle[0]

          msg.pose = Pose()
          msg.pose.position.x = obstacle[1][0]
          msg.pose.position.y = obstacle[1][1]
          msg.pose.position.z = obstacle[1][2]

          self.agent_topic_str+=f',{self._ns_prefix}pedsim_static_obstacle_{obstacle[0]}/0' 
          msg.type = "shelf"
          msg.interaction_radius = 0.0
          msg.yaml_path = os.path.join(
              rospkg.RosPack().get_path("arena-simulation-setup"),
              "obstacles", "shelf.yaml"
          )
          srv.InteractiveObstacles.append(msg)
          i = i+1

      max_num_try = 2
      i_curr_try = 0
      print("trying to call service with interactive obstacles: ")    

      while i_curr_try < max_num_try:
      # try to call service
          response=self.__respawn_interactive_obstacles_srv.call(srv.InteractiveObstacles)

          if not response.success:  # if service not succeeds, do something and redo service
              rospy.logwarn(
                  f"spawn human failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
              # rospy.logwarn(response.message)
              i_curr_try += 1
          else:
              break
      # self._peds = peds
      # self.spawned_obstacles.append(srv.InteractiveObstacles)
      rospy.set_param(f'{self._ns_prefix}agent_topic_string', self.agent_topic_str)
      rospy.set_param("respawn_static", True)
      return

    # # WORK IN PROGRESS
    # num_obstacles = 1
    # model_yaml_file_path = os.path.join("../utils/arena-simulation-setup/obstacles", "shelf.yaml")
    # start_pos = obstacles[0][1]
    # print(obstacles)
    # vertices = np.array([[0, 0], [0, 0], [0, 0], [0, 0]])
    # type_obstacle = "static"

    # max_num_try = 2
    # i_curr_try = 0
    # while i_curr_try < max_num_try:
    #     spawn_request = SpawnModelRequest()
    #     # spawn_request.yaml_path = model_yaml_file_path
    #     spawn_request.model_name = "0"
    #     # x, y, theta = get_random_pos_on_map(self._free_space_indices, self.map,)
    #     # set the postion of the obstacle out of the ma4p to hidden them
    #     if len(start_pos) == 0:
    #         x = self.map.info.origin.position.x - 3 * \
    #             self.map.info.resolution * self.map.info.height
    #         y = self.map.info.origin.position.y - 3 * \
    #             self.map.info.resolution * self.map.info.width
    #         theta = theta = random.uniform(-math.pi, math.pi)
    #     else:
    #         # assert len(start_pos) == 3
    #         x = start_pos[0]
    #         y = start_pos[1]
    #         theta = start_pos[2]
    #     spawn_request.initial_pose = Pose(Point(x,y,0), Quaternion(0,0,0,0))
    #     print(spawn_request)
    #     # spawn_request.initial_pose.x = x
    #     # spawn_request.initial_pose.y = y
    #     # spawn_request.initial_pose.theta = theta
    #     # try to call service
    #     # response = self._srv_spawn_model.call(spawn_request)
    #     # if not response.success:  # if service not succeeds, do something and redo service
    #     #     rospy.logwarn(
    #     #         f"({self.ns}) spawn object failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
    #     #     rospy.logwarn(response.message)
    #     #     i_curr_try += 1
    #     # else:
    #     # self.obstacle_name_list.append(spawn_request.model_name)
    #     #tell the info of polygon obstacles to pedsim
    #     add_pedsim_srv=SpawnObstacleRequest()
    #     for i in range(size):
    #         lineObstacle=LineObstacle()
    #         lineObstacle.start.x,lineObstacle.start.y=obstacles[i,0,0],obstacles[i,0,1]
    #         lineObstacle.end.x,lineObstacle.end.y=obstacles[i,1,0],obstacles[i,1,1]
    #         add_pedsim_srv.staticObstacles.obstacles.append(lineObstacle)
    #     self.__add_obstacle_srv.call(add_pedsim_srv)
    #     break
    # if i_curr_try == max_num_try:
    #     raise rospy.ServiceException(f"({self.ns}) failed to register obstacles")
    # return

  def spawn_pedsim_interactive_obstacles(self, obstacles):
      srv = SpawnInteractiveObstacles()
      srv.InteractiveObstacles = []
      i = 0
      self.agent_topic_str=''   
      while i < len(obstacles) : 
          msg = InteractiveObstacle()
          obstacle = obstacles[i]
          # msg.id = obstacle[0]

          msg.pose = Pose()
          msg.pose.position.x = obstacle[1][0]
          msg.pose.position.y = obstacle[1][1]
          msg.pose.position.z = obstacle[1][2]

          self.agent_topic_str+=f',{self._ns_prefix}pedsim_interactive_obstacle_{obstacle[0]}/0' 
          msg.type = "shelf"
          msg.interaction_radius = 1.0
          msg.yaml_path = os.path.join(
              rospkg.RosPack().get_path("arena-simulation-setup"),
              "obstacles", "shelf.yaml"
          )
          srv.InteractiveObstacles.append(msg)
          i = i+1

      max_num_try = 2
      i_curr_try = 0
      print("trying to call service with interactive obstacles: ")    

      while i_curr_try < max_num_try:
      # try to call service
          # response=self.__respawn_interactive_obstacles_srv.call(srv.InteractiveObstacles)
          response=self.spawn_interactive_obstacles_srv.call(srv.InteractiveObstacles)

          if not response.success:  # if service not succeeds, do something and redo service
              rospy.logwarn(
                  f"spawn human failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
              # rospy.logwarn(response.message)
              i_curr_try += 1
          else:
              break
      # self._peds = peds
      # self._peds.append(srv.InteractiveObstacles)
      # self.spawned_obstacles.append(srv.InteractiveObstacles)
      rospy.set_param(f'{self._ns_prefix}agent_topic_string', self.agent_topic_str)
      rospy.set_param("respawn_interactive", True)
      return

  def spawn_pedsim_dynamic_obstacles(self, peds):
      srv = SpawnPeds()
      srv.peds = []
      i = 0
      self.agent_topic_str=''   
      while i < len(peds) : 
          msg = Ped()
          ped = peds[i]
          msg.id = ped[0]

          msg.pos = Point()
          msg.pos.x = ped[1][0]
          msg.pos.y = ped[1][1]
          msg.pos.z = ped[1][2]

          self.agent_topic_str+=f',pedsim_agent_{ped[0]}/0' 
          msg.type = "adult"
          msg.yaml_file = os.path.join(
              rospkg.RosPack().get_path("arena-simulation-setup"),
              "dynamic_obstacles",
              "person_two_legged.model.yaml"
          )
          msg.number_of_peds = 1
          msg.vmax = 0.3
          msg.start_up_mode = "default"
          msg.wait_time = 0.0
          msg.trigger_zone_radius = 0.0
          msg.chatting_probability = 0.00
          msg.tell_story_probability = 0
          msg.group_talking_probability = 0.00
          msg.talking_and_walking_probability = 0.00
          msg.requesting_service_probability = 0.00
          msg.requesting_guide_probability = 0.00
          msg.requesting_follower_probability = 0.00
          msg.max_talking_distance = 5
          msg.max_servicing_radius = 5
          msg.talking_base_time = 10
          msg.tell_story_base_time = 0
          msg.group_talking_base_time = 10
          msg.talking_and_walking_base_time = 6
          msg.receiving_service_base_time = 20
          msg.requesting_service_base_time = 30
          msg.force_factor_desired = 1
          msg.force_factor_obstacle = 1
          msg.force_factor_social = 5
          msg.force_factor_robot = 1
          msg.waypoint_mode = 0 # or 1 check later

          msg.waypoints = []

          for pos in ped[2]:
              p = Point()
              p.x = pos[0]
              p.y = pos[1]
              p.z = pos[2]
              msg.waypoints.append(p)
          srv.peds.append(msg)
          i = i+1

      max_num_try = 2
      i_curr_try = 0
      while i_curr_try < max_num_try:
      # try to call service
          response=self.__respawn_peds_srv.call(srv.peds)

          if not response.success:  # if service not succeeds, do something and redo service
              rospy.logwarn(
                  f"spawn human failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
              i_curr_try += 1
          else:
              break
      # self.__peds = peds
      # self._peds.append(srv.peds)
      # self.spawned_obstacles.append(srv.peds)
      rospy.set_param(f'{self._ns_prefix}agent_topic_string', self.agent_topic_str)
      rospy.set_param("respawn_dynamic", True)

      # USE THE FOLLOWING CODE TO SPAWN ACTORS WITHOUT PEDSIM

      # self.agent_topic_str=''  
      # for ped in peds: 
      #     id, pose, waypoints = ped
      #     x, y, theta = pose

      #     rospy.loginfo("Spawning model: actor_id = %s", id)

      #     model_pose = Pose(Point(x=x,
      #                           y=y,
      #                           z=0), Quaternion())
      #     new_xml_string= self.xml_string.replace("0 0 0.75", str(x) + " " + str(y) +" 0.75")
      #     new_xml_string= new_xml_string.replace("actor2", str(id))
      #     new_xml_string= new_xml_string.replace(
      #        "__waypoints__", 
      #        "".join(
      #           [
      #              f"<waypoint>{x} {y} {theta}</waypoint>" for x, y, theta in waypoints
      #             ]
      #           )
      #         )

      #     self.spawn_model(str(id), new_xml_string, "", model_pose, "world")
      #     # self.spawn_model(actor_id, self.xml_string, "", model_pose, "world")
      #     rospy.set_param("respawn_dynamic", False)

      # self._peds = peds
      # rospy.set_param(f'{self._ns_prefix}agent_topic_string', self.agent_topic_str)
      return
      
  def spawn_pedsim_map_borders(self):
    map_service = rospy.ServiceProxy("/static_map", GetMap)
    self.map = map_service().map
    self._free_space_indices = Utils.update_freespace_indices_maze(self.map)
    border_vertex=Utils.generate_map_inner_border(self._free_space_indices,self.map)

    self.map_border_vertices=border_vertex
    add_pedsim_srv=SpawnObstacleRequest()
    size=border_vertex.shape[0]
    for i in range(size):
        lineObstacle=LineObstacle()
        lineObstacle.start.x,lineObstacle.start.y=border_vertex[i,0],border_vertex[i,1]
        lineObstacle.end.x,lineObstacle.end.y=border_vertex[(i+1)%size,0],border_vertex[(i+1)%size,1]
        add_pedsim_srv.staticObstacles.obstacles.append(lineObstacle)
    self.__add_obstacle_srv.call(add_pedsim_srv)

  def spawn_pedsim_map_obstacles(self):
    map = rospy.get_param("map_file")
    print("READING XML")
    map_path = os.path.join(
        rospkg.RosPack().get_path("arena-simulation-setup"), 
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
  def spawn_pedsim_dynamic_scenario_obstacles(self, peds):
      srv = SpawnPeds()
      srv.peds = []
      i = 0
      self.agent_topic_str=''   
      while i < len(peds) : 
        ped = peds[i]
        msg = Ped()
        msg.id = i 

        msg.pos = Point()
        msg.pos.x = ped["pos"][0]
        msg.pos.y = ped["pos"][1]
        msg.pos.z = 0

        msg.waypoints = []
        for pos in ped["waypoints"]:
            p = Point()
            p.x = pos[0]
            p.y = pos[1]
            p.z = 0
            msg.waypoints.append(p)
        msg.yaml_file = os.path.join(
            rospkg.RosPack().get_path("arena-simulation-setup"),
            "dynamic_obstacles",
            "person_two_legged.model.yaml"
          )

        self.agent_topic_str+=f',pedsim_agent_{i}/0' 
        msg.type = "adult"
        msg.number_of_peds = 1
        # msg.vmax = 0.3
        msg.vmax = ped["vmax"]
        msg.start_up_mode = ped["start_up_mode"]
        msg.wait_time = ped["wait_time"]
        msg.trigger_zone_radius = ped["trigger_zone_radius"]
        msg.chatting_probability = ped["chatting_probability"]
        msg.tell_story_probability = ped["tell_story_probability"]
        msg.group_talking_probability = ped["group_talking_probability"]
        msg.talking_and_walking_probability = ped["talking_and_walking_probability"]
        msg.requesting_service_probability = ped["requesting_service_probability"]
        msg.requesting_guide_probability = ped["requesting_guide_probability"]
        msg.requesting_follower_probability = ped["requesting_follower_probability"]
        msg.max_talking_distance = ped["max_talking_distance"]
        msg.max_servicing_radius = ped["max_servicing_radius"]
        msg.talking_base_time = ped["talking_base_time"]
        msg.tell_story_base_time = ped["tell_story_base_time"]
        msg.group_talking_base_time = ped["group_talking_base_time"]
        msg.talking_and_walking_base_time = ped["talking_and_walking_base_time"]
        msg.receiving_service_base_time = ped["receiving_service_base_time"]
        msg.requesting_service_base_time = ped["requesting_service_base_time"]
        msg.force_factor_desired = ped["force_factor_desired"]
        msg.force_factor_obstacle = ped["force_factor_obstacle"]
        msg.force_factor_social = ped["force_factor_social"]
        msg.force_factor_robot = ped["force_factor_robot"]
        msg.waypoint_mode = ped["waypoint_mode"] # or 1 check later

        srv.peds.append(msg)
        i = i+1

      max_num_try = 2
      i_curr_try = 0
      while i_curr_try < max_num_try:
      # try to call service
          response=self.__respawn_peds_srv.call(srv.peds)

          if not response.success:  # if service not succeeds, do something and redo service
              rospy.logwarn(
                  f"spawn human failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
              # rospy.logwarn(response.message)
              i_curr_try += 1
          else:
              break
      self._peds = peds
      rospy.set_param(f'{self._ns_prefix}agent_topic_string', self.agent_topic_str)
      return

  def spawn_pedsim_interactive_scenario_obstacles(self, obstacles):
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

          self.agent_topic_str+=f',{self._ns_prefix}pedsim_static_obstacle_{i}/0' 
          msg.type = "shelf"
          msg.interaction_radius = 0.0
          msg.yaml_path = os.path.join(
              rospkg.RosPack().get_path("arena-simulation-setup"),
              "obstacles", "shelf.yaml"
          )
          srv.InteractiveObstacles.append(msg)
          i = i+1

      max_num_try = 2
      i_curr_try = 0
      print("trying to call service with static obstacles: ")    

      while i_curr_try < max_num_try:
      # try to call service
          response=self.__respawn_interactive_obstacles_srv.call(srv.InteractiveObstacles)

          if not response.success:  # if service not succeeds, do something and redo service
              rospy.logwarn(
                  f"spawn human failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
              # rospy.logwarn(response.message)
              i_curr_try += 1
          else:
              break
      # self._peds.append(peds)
      rospy.set_param(f'{self._ns_prefix}agent_topic_string', self.agent_topic_str)
      return

  def remove_all_obstacles(self):
    if rospy.get_param("pedsim"):
    #   self._remove_peds_srv(True)
      print("************************trying to remove all interactive obstacles")
      self.__remove_all_interactive_obstacles_srv.call()
    # # Anhand ID gazebo obstacles löschen
    # print("REMOVE ALL OBSTACLES (currently not working)")
    # print("spawned obstacles: ", self.spawned_obstacles)

    for ped in self.spawned_obstacles:
      print("remove obstacle", ped)
      self.remove_model_srv(str(ped))

    # rospy.set_param("respawn_dynamic", True)
    # rospy.set_param("respawn_static", True)
    # rospy.set_param("respawn_interactive", True)

    self.spawned_obstacles = []

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