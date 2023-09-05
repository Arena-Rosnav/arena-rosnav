from abc import abstractmethod
from typing import List, Tuple

import rospy
import numpy as np
import os
import yaml
import math
import rospkg
import random
from flatland_msgs.srv import (
    MoveModelRequest,
    MoveModel,
    SpawnModel,
    SpawnModels,
    DeleteModel,
    DeleteModels,
    DeleteModelsRequest,
    SpawnModelRequest,
    SpawnModelsRequest
)
from geometry_msgs.msg import Pose2D, Pose
from flatland_msgs.msg import MoveModelMsg, Model
from nav_msgs.srv import GetMap
from pedsim_srvs.srv import SpawnPeds
from pedsim_msgs.msg import Ped
from pedsim_srvs.srv import SpawnInteractiveObstacles,SpawnInteractiveObstaclesRequest
from pedsim_srvs.srv import SpawnObstacle,SpawnObstacleRequest
from pedsim_msgs.msg import InteractiveObstacle, LineObstacle



from task_generator.manager.pedsim_manager import PedsimManager
from task_generator.utils import Utils
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

from ..constants import Constants, FlatlandRandomModel, Pedsim
from .base_simulator import BaseSimulator
from .simulator_factory import SimulatorFactory





T = Constants.WAIT_FOR_SERVICE_TIMEOUT


@SimulatorFactory.register("flatland")
class FlatlandSimulator(BaseSimulator):
    """
    This is the flatland encoder for connecting
    flatland with the arena-benchmark task
    generator. The class implements all methods
    defined in `BaseSimulator`.

    For flatland to work properly, a dedicated .yaml
    file has to be created for each used model. This
    is, because the spawn model request only contains
    the path to this file instead of the file content
    directly. For each reset a new set of obstacles
    is created and saved in files. The path to these
    files is defined in the `tmp_model_path` param
    and defaults to `/tmp`.
    """

    PLUGIN_PROPS_TO_EXTEND = {
        "DiffDrive": ["odom_pub", "twist_sub"],
        "Laser": ["topic"],
    }

    def __init__(self, namespace):
        super().__init__(namespace)
        self._namespace = namespace
        self._ns_prefix = "" if namespace == "" else "/" + namespace + "/"

        self._move_robot_pub = rospy.Publisher(
            self._ns_prefix + "move_model", MoveModelMsg, queue_size=10
        )

        self._robot_name = rospy.get_param("robot_model", "")
        self._robot_radius = rospy.get_param("robot_radius", "")
        self._is_training_mode = rospy.get_param("train_mode", "")
        self._step_size = rospy.get_param("step_size", "")
        # self._robot_yaml_path = rospy.get_param("robot_yaml_path")
        self._tmp_model_path = rospy.get_param("tmp_model_path", "/tmp")

        rospy.wait_for_service(f"{self._ns_prefix}move_model", timeout=T)
        rospy.wait_for_service(f"{self._ns_prefix}spawn_model", timeout=T)
        rospy.wait_for_service(f"{self._ns_prefix}delete_model", timeout=T)
        rospy.wait_for_service(f"{self._ns_prefix}pedsim_simulator/respawn_peds" , timeout=20)
        rospy.wait_for_service(f"{self._ns_prefix}pedsim_simulator/respawn_interactive_obstacles" , timeout=20)
        rospy.wait_for_service(f'{self._ns_prefix}pedsim_simulator/add_obstacle', timeout=20)

        self._move_model_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}move_model", MoveModel, persistent=True
        )
        self._spawn_model_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}spawn_model", SpawnModel
        )
        self._spawn_model_from_string_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}spawn_model_from_string", SpawnModel
        )
        self._spawn_models_from_string_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}spawn_models_from_string", SpawnModels
        )
        self._delete_model_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}delete_model", DeleteModel
        )
        self._delete_models_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}delete_models", DeleteModels
        )
        self.__respawn_peds_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}pedsim_simulator/respawn_peds" , SpawnPeds, persistent=True)

        self._spawn_peds_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}pedsim_simulator/spawn_peds", SpawnPeds
        )
        self._reset_peds_srv = rospy.ServiceProxy(
            f"{self._ns_prefix}pedsim_simulator/reset_all_peds", Trigger
        )
        self.__add_obstacle_srv = rospy.ServiceProxy(
            f'{self._ns_prefix}pedsim_simulator/add_obstacle' ,SpawnObstacle, persistent=True)

        self.__respawn_interactive_obstacles_srv = rospy.ServiceProxy(
        f"{self._ns_prefix}pedsim_simulator/respawn_interactive_obstacles" ,SpawnInteractiveObstacles, persistent=True)

        self.obs_names = []

    @property
    def obstacles_amount(self):
        return len(self.obs_names)

    def add_obs_to_list(self, obs_name: str):
        self.obs_names.append(obs_name)

    def before_reset_task(self):
        pass

    def after_reset_task(self):
        pass

    # PEDSIM

    def remove_all_obstacles(self):
        request = DeleteModelsRequest()
        request.name = self.obs_names

        self._delete_models_srv(request)

        self.obs_names = []

    def spawn_pedsim_agents(self, dynamic_obstacles: List):
        
        if len(dynamic_obstacles) <= 0:
            return
        
        peds = [
            FlatlandSimulator.create_ped_msg(p, i)
            for i, p in enumerate(dynamic_obstacles)
        ]

        spawn_ped_msg = SpawnPeds()

        spawn_ped_msg.peds = peds

        self._spawn_peds_srv(peds)

    def reset_pedsim_agents(self):
        self._reset_peds_srv()

    # CREATE OBSTACLES

    # def spawn_random_ped_test(self, **args):
    #     peds = [create_ped_msg(p, i) for i, p in enumerate(dynamic_obstacles)]

    #     spawn_ped_msg = SpawnPeds()

    #     spawn_ped_msg.peds = peds

    #     self._spawn_peds_srv(spawn_ped_msg)
    #     return 0

    def create_dynamic_obstacle(self, **args):
        return self._create_obstacle(**args, is_dynamic=True)


    def create_static_obstacle(self, **args):
        return self._create_obstacle(**args, is_dynamic=False)

    def _create_obstacle(self, is_dynamic=False, position=None, **args):
        if position is None:
            position = [0, 0, 0]

        model = self._generate_random_obstacle(is_dynamic=is_dynamic, **args)
        name = FlatlandSimulator.create_obs_name(self.obstacles_amount)

        self.add_obs_to_list(name)

        return yaml.dump(model), name, position

    # SPAWN OBSTACLES
    def spawn_obstacle(
        self, position: Tuple[int, int, int], yaml_path=""
    ):
        name = FlatlandSimulator.create_obs_name(self.obstacles_amount)

        try:
            self._spawn_model(yaml_path, name, self._namespace, position)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to spawn model {name} with exception: {e}")

        self.add_obs_to_list(name)

    def spawn_obstacles(self, obstacles):

        request = SpawnModelsRequest()

        models = []

        for obstacle in obstacles:
            m = Model()
            m.yaml_path = obstacle[0]
            m.name = obstacle[1]
            m.ns = self._namespace
            m.pose.x = obstacle[2][0]
            m.pose.y = obstacle[2][1]
            m.pose.theta = obstacle[2][2]

            models.append(m)

        request.models = models

        self._spawn_models_from_string_srv(request)


    # PEDSIM INTEGRATION 


   
    
    def create_pedsim_static_obstacle(self, i, map_manager, forbidden_zones):
        num_obstacles = 1
        model_yaml_file_path = os.path.join("../utils/arena-simulation-setup/obstacles", "random.model.yaml")
        start_pos = []
        vertices = np.array([[0, 0], [0, 0], [0, 0], [0, 0]])
        type_obstacle = "static"

        max_num_try = 2
        i_curr_try = 0
        while i_curr_try < max_num_try:
            spawn_request = SpawnModelRequest()
            spawn_request.yaml_path = model_yaml_file_path
            spawn_request.name = f'{name_prefix}_{instance_idx:02d}'
            # x, y, theta = get_random_pos_on_map(self._free_space_indices, self.map,)
            # set the postion of the obstacle out of the map to hidden them
            if len(start_pos) == 0:
                x = self.map.info.origin.position.x - 3 * \
                    self.map.info.resolution * self.map.info.height
                y = self.map.info.origin.position.y - 3 * \
                    self.map.info.resolution * self.map.info.width
                theta = theta = random.uniform(-math.pi, math.pi)
            else:
                assert len(start_pos) == 3
                x = start_pos[0]
                y = start_pos[1]
                theta = start_pos[2]
            spawn_request.pose.x = x
            spawn_request.pose.y = y
            spawn_request.pose.theta = theta
            # try to call service
            response = self._srv_spawn_model.call(spawn_request)
            if not response.success:  # if service not succeeds, do something and redo service
                rospy.logwarn(
                    f"({self.ns}) spawn object {spawn_request.name} failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
                rospy.logwarn(response.message)
                i_curr_try += 1
            else:
                self.obstacle_name_list.append(spawn_request.name)
                #tell the info of polygon obstacles to pedsim
                add_pedsim_srv=SpawnObstacleRequest()
                size=vertices.shape[0]
                for i in range(size):
                    lineObstacle=LineObstacle()
                    lineObstacle.start.x,lineObstacle.start.y=vertices[i,0],vertices[i,1]
                    lineObstacle.end.x,lineObstacle.end.y=vertices[(i+1)%size,0],vertices[(i+1)%size,1]
                    add_pedsim_srv.staticObstacles.obstacles.append(lineObstacle)
                self.__add_obstacle_srv.call(add_pedsim_srv)
                break
        if i_curr_try == max_num_try:
            raise rospy.ServiceException(f"({self.ns}) failed to register obstacles")
        
        return 

    def create_pedsim_interactive_obstacle(self, i, map_manager, forbidden_zones):
        # print("305 safe")
        self.map_manager = map_manager
        ped_array =np.array([],dtype=object).reshape(0,3) # Not used
        # self.human_id+=1
        safe_distance = 3.5

        [x, y, theta] = self.map_manager.get_random_pos_on_map(safe_distance, forbidden_zones) # check later for the need of free indicies and map papram
        # print(obstacles[i])
        # if random.uniform(0.0, 1.0) < 0.8:
        ped=np.array([i+1, [x, y, 0.0]],dtype=object)
        # print("323 safe")
        
        return ped

        # self.create_ped_msg(ped_array, id)

    def create_pedsim_dynamic_obstacle(self,i, map_manager, forbidden_zones):
        # print("305 safe")
        self.map_manager = map_manager
        ped_array =np.array([],dtype=object).reshape(0,3) # Not used
        # self.human_id+=1
        safe_distance = 3.5

        [x, y, theta] = self.map_manager.get_random_pos_on_map(safe_distance, forbidden_zones) # check later for the need of free indicies and map papram
        # print(obstacles[i])
        # if random.uniform(0.0, 1.0) < 0.8:
        waypoints = np.array( [x, y, 1]).reshape(1, 3) # the first waypoint
        safe_distance = 0.1 # the other waypoints don't need to avoid robot
        # print("316 safe")
        for j in range(10): # noote was 1000
            dist = 0
            while dist < 8:
                [x2, y2, theta2] = self.map_manager.get_random_pos_on_map( safe_distance, forbidden_zones)
                dist = np.linalg.norm([waypoints[-1,0] - x2,waypoints[-1,1] - y2])
            waypoints = np.vstack([waypoints, [x2, y2, 1]])
        ped=np.array([i+1, [x, y, 0.0], waypoints],dtype=object)
        # print("323 safe")
        
        return ped
    
    def spawn_pedsim_static_obstacles(self, obstacles):
        # TODO adjust if necessary
        # _add_map_border_in_pedsim
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

        

        return

    def spawn_pedsim_interactive_obstacles(self, obstacles):
        print("225spawning pedsim dynamic obstacles")
        # print(peds.shape)

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
            # msg.name = "test"
            msg.interaction_radius = 0.0
            msg.yaml_path = os.path.join(
                rospkg.RosPack().get_path("arena-simulation-setup"),
                "obstacles", "long_shelf.model.yaml"
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
        # self.__peds = peds
        rospy.set_param(f'{self._ns_prefix}agent_topic_string', self.agent_topic_str)
        return

    def spawn_pedsim_dynamic_obstacles(self, peds):
        print("225spawning pedsim dynamic obstacles")
        # print(peds.shape)

        srv = SpawnPeds()
        srv.peds = []
        i = 0
        self.agent_topic_str=''   
        while i < len(peds) : 
            msg = Ped()
            ped = peds[i]
            print("printing a ped 238")
            print(len(ped))
            print(ped)
            msg.id = ped[0]

            msg.pos = Point()
            msg.pos.x = ped[1][0]
            msg.pos.y = ped[1][1]
            msg.pos.z = ped[1][2]

            self.agent_topic_str+=f',{self._ns_prefix}pedsim_agent_{ped[0]}/0' 
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
            print("275spawning pedsim dynamic obstacles")

            for pos in ped[2]:
                p = Point()
                p.x = pos[0]
                p.y = pos[1]
                p.z = pos[2]
                msg.waypoints.append(p)
            srv.peds.append(msg)
            i = i+1
            print("285spawning pedsim dynamic obstacles")

        max_num_try = 2
        i_curr_try = 0
        print("trying to call service with peds: ")    
        print(peds)    
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
        self.__peds = peds
        rospy.set_param(f'{self._ns_prefix}agent_topic_string', self.agent_topic_str)
        return
    # ROBOT

    def spawn_robot(self, name, robot_name, namespace_appendix=None, complexity=1):
        base_model_path = os.path.join(
            rospkg.RosPack().get_path("arena-simulation-setup"),
            "robot",
            robot_name,
        )

        yaml_path = os.path.join(base_model_path, robot_name + ".model.yaml")

        file_content = self._update_plugin_topics(self._read_yaml(yaml_path), name)

        self._spawn_model(
            yaml.dump(file_content),
            name,
            os.path.join(self._namespace, namespace_appendix)
            if len(namespace_appendix) > 0
            else self._namespace,
            [0, 0, 0],
            srv=self._spawn_model_from_string_srv,
        )

    def move_robot(self, pos, name=None):
        pose = Pose2D()
        pose.x = pos[0]
        pose.y = pos[1]
        pose.theta = pos[2]

        move_model_request = MoveModelRequest()
        move_model_request.name = name or self._robot_name
        move_model_request.pose = pose

        self._move_model_srv(move_model_request)

    # UTILS

    def _spawn_model(
        self,
        yaml_path: str,
        name: str,
        namespace: str,
        position: Tuple[int, int, int],
        srv=rospy.ServiceProxy,
    ):
        request = SpawnModelRequest()
        request.yaml_path = yaml_path
        request.name = name
        request.ns = namespace
        request.pose.x = position[0]
        request.pose.y = position[1]
        request.pose.theta = position[2]

        if srv is None:
            srv = self._spawn_model_srv

        srv(request)

    def _generate_random_obstacle(
        self,
        is_dynamic=False,
        min_radius=FlatlandRandomModel.MIN_RADIUS,
        max_radius=FlatlandRandomModel.MAX_RADIUS,
        linear_vel=FlatlandRandomModel.LINEAR_VEL,
        angular_vel_max=FlatlandRandomModel.ANGLUAR_VEL_MAX,
    ):
        """
        Creates a dict in the flatland model schema.

        Since a lot of the variables are untouched
        the majority of the dict is filled up with
        constants defined in the `Constants` file.
        """
        body = {
            **FlatlandRandomModel.BODY,
            "type": "dynamic" if is_dynamic else "static",
        }

        footprint = {
            **FlatlandRandomModel.FOOTPRINT,
            **self._generate_random_footprint_type(min_radius, max_radius),
        }

        body["footprints"] = [footprint]

        model = {"bodies": [body], "plugins": []}

        if is_dynamic:
            model["plugins"].append(
                {
                    **FlatlandRandomModel.RANDOM_MOVE_PLUGIN,
                    "linear_velocity": random.uniform(0, linear_vel),
                    "angular_velocity_max": angular_vel_max,
                }
            )

        return model

    def _generate_random_footprint_type(self, min_radius, max_radius):
        """
        An object in flatland can either be a circle with a
        specific radius or a polygon shape.

        This function will choose a shape randomly and
        creates a shape from this.

        For the circle the radius is chosen randomly and
        lies in a specific range defined in the `constants` file

        For the polygon, the amount of vertexes is determined
        at first. Then the vertexes are distributed around the center
        and for each vertex a distance to the center is calculated.
        At the end, the vertexes form the polygon. The distance
        to the center is chosen randomly and lies in the range
        defined in `constants`.
        """
        type = random.choice(["circle", "polygon"])

        if type == "circle":
            radius = random.uniform(min_radius, max_radius)

            return {"type": type, "radius": radius}

        points_amount = random.randint(3, 8)  # Defined in flatland definition
        angle_interval = 2 * np.pi / points_amount

        points = []

        for p in range(points_amount):
            angle = random.uniform(0, angle_interval)
            radius = random.uniform(min_radius, max_radius)

            real_angle = angle_interval * p + angle

            points.append(
                [math.cos(real_angle) * radius, math.sin(real_angle) * radius]
            )

        return {"type": type, "points": list(points)}

    def _create_obstacle_yaml(self, model, obs_name):
        os.makedirs(self._tmp_model_path, exist_ok=True)

        tmp_model_file_name = self._namespace + "_" + obs_name + ".model.yaml"

        model_file_name = self._tmp_model_path + "/" + tmp_model_file_name

        with open(model_file_name, "w") as fd:
            yaml.dump(model, fd)

        return model_file_name

    def _update_plugin_topics(self, file_content, namespace):
        if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
            return file_content

        plugins = file_content["plugins"]

        for plugin in plugins:
            if FlatlandSimulator.PLUGIN_PROPS_TO_EXTEND.get(plugin["type"]):
                prop_names = FlatlandSimulator.PLUGIN_PROPS_TO_EXTEND.get(
                    plugin["type"]
                )

                for name in prop_names:
                    plugin[name] = os.path.join(namespace, plugin[name])

        return file_content

    def _read_yaml(self, yaml_path):
        with open(yaml_path, "r") as file:
            return yaml.safe_load(file)

    @staticmethod
    def create_obs_name(number: int):
        return f"obs_{number}"

    @staticmethod
    def check_yaml_path(path):
        return os.path.isfile(path)

    @staticmethod
    def create_ped_msg(ped, id):
        msg = Ped()

        msg.id = id

        pos = Point()
        pos.x = ped["waypoints"][0][0]
        pos.y = ped["waypoints"][0][1]
        msg.pos = pos

        msg.type = "adult"
        msg.yaml_file = os.path.join(
            rospkg.RosPack().get_path("arena-simulation-setup"),
            "dynamic_obstacles",
            "person_two_legged.model.yaml",
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

        waypoints = []

        for w in ped["waypoints"]:
            new_waypoint = Point()

            new_waypoint.x = w[0]
            new_waypoint.y = w[1]

            waypoints.append(new_waypoint)

        msg.waypoints = waypoints

        msg.waypoint_mode = 0

        return msg