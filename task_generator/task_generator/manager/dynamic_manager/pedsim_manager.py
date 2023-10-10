import itertools
from task_generator.manager.dynamic_manager.dynamic_manager import DynamicManager
from task_generator.simulators.base_simulator import BaseSimulator

import os

from pedsim_msgs.msg import Ped
from geometry_msgs.msg import Point
from task_generator.constants import Pedsim


import rospy
import re
from scipy.spatial.transform import Rotation


from pedsim_srvs.srv import SpawnInteractiveObstacles, SpawnObstacle, SpawnPeds
from pedsim_msgs.msg import InteractiveObstacle, AgentStates, Waypoints, Ped

from geometry_msgs.msg import Point, Pose, Quaternion

from std_srvs.srv import SetBool, Trigger

from rospkg import RosPack

from task_generator.constants import Constants, Pedsim

from task_generator.shared import Model, ModelType, ModelWrapper, Obstacle
from task_generator.utils import ModelLoader


T = Constants.WAIT_FOR_SERVICE_TIMEOUT


class PedsimManager(DynamicManager):

    _id_gen: itertools.count

    _spawn_peds_srv: rospy.ServiceProxy
    _remove_peds_srv: rospy.ServiceProxy
    _reset_peds_srv: rospy.ServiceProxy
    _respawn_interactive_obstacles_srv: rospy.ServiceProxy
    _remove_all_interactive_obstacles_srv: rospy.ServiceProxy
    _spawn_interactive_obstacles_srv: rospy.ServiceProxy
    _respawn_peds_srv: rospy.ServiceProxy
    _add_obstacle_srv: rospy.ServiceProxy

    _xml_string: str

    #TODO unclean
    _pedsim_model_loader: ModelLoader

    def __init__(self, namespace: str, simulator: BaseSimulator):

        super().__init__(namespace, simulator)

        self._id_gen = itertools.count(20)

        rospy.wait_for_service("/pedsim_simulator/spawn_peds", timeout=T)
        rospy.wait_for_service("/pedsim_simulator/reset_all_peds", timeout=T)
        rospy.wait_for_service("/pedsim_simulator/remove_all_peds", timeout=T)
        rospy.wait_for_service("pedsim_simulator/respawn_peds", timeout=T)
        rospy.wait_for_service(
            "pedsim_simulator/respawn_interactive_obstacles", timeout=T)
        rospy.wait_for_service(
            "pedsim_simulator/remove_all_interactive_obstacles", timeout=T)
        rospy.wait_for_service("pedsim_simulator/add_obstacle", timeout=T)

        self._spawn_peds_srv = rospy.ServiceProxy(
            "/pedsim_simulator/spawn_peds", SpawnPeds
        )
        self._remove_peds_srv = rospy.ServiceProxy(
            "/pedsim_simulator/remove_all_peds", SetBool
        )
        self._reset_peds_srv = rospy.ServiceProxy(
            "/pedsim_simulator/reset_all_peds", Trigger
        )
        self._respawn_interactive_obstacles_srv = rospy.ServiceProxy(
            "pedsim_simulator/respawn_interactive_obstacles", SpawnInteractiveObstacles, persistent=True)

        self._remove_all_interactive_obstacles_srv = rospy.ServiceProxy(
            "pedsim_simulator/remove_all_interactive_obstacles", Trigger)

        self._spawn_interactive_obstacles_srv = rospy.ServiceProxy(
            "pedsim_simulator/spawn_interactive_obstacles", SpawnInteractiveObstacles, persistent=True)

        self._respawn_peds_srv = rospy.ServiceProxy(
            "pedsim_simulator/respawn_peds", SpawnPeds, persistent=True)

        self._add_obstacle_srv = rospy.ServiceProxy(
            "pedsim_simulator/add_obstacle", SpawnObstacle, persistent=True)

        rospy.set_param("respawn_dynamic", True)
        rospy.set_param("respawn_static", True)
        rospy.set_param("respawn_interactive", True)
        rospy.Subscriber("/pedsim_simulator/simulated_waypoints",
                         Waypoints, self._interactive_actor_poses_callback)
        rospy.Subscriber("/pedsim_simulator/simulated_agents",
                         AgentStates, self._dynamic_actor_poses_callback)

        # override
        self._xml_string = """
            <?xml version="1.0" ?>
            <sdf version="1.5">
            <model name="actor_model">
                <pose>0 0 0.75 0 0 0</pose>
                <link name="link">
                    <collision name="collision">
                    <geometry>
                        <box>
                        <size>.5 .5 1.5</size>
                        </box>
                    </geometry>
                    </collision>
                    <visual name="visual">
                    <geometry>
                        <box>
                        <size>.5 .5 1.5</size>
                        </box>
                    </geometry>
                    </visual>
                </link>
            </model>
        """

    def spawn_obstacles(self, obstacles):
        srv = SpawnInteractiveObstacles()
        srv.InteractiveObstacles = []

        self.agent_topic_str = ''
        for obstacle in obstacles:
            msg = InteractiveObstacle()

            #TODO create a global helper function for this kind of use case
            msg.pose = Pose(
                position=Point(x=obstacle.position[0], y=obstacle.position[1], z=0),
                orientation=Quaternion(x=0, y=0, z=obstacle.position[2], w=1)
            )

            interaction_radius: float = obstacle.extra.get(
                "interaction_radius", 0.)

            if interaction_radius > 0.:
                self.agent_topic_str += f',{self._ns_prefix()}pedsim_static_obstacle_{obstacle.name}/0'
            else:
                self.agent_topic_str += f',{self._ns_prefix()}pedsim_interactive_obstacle_{obstacle.name}/0'

            msg.type = obstacle.extra.get("type","")
            msg.interaction_radius = interaction_radius

            #TODO feed the content from a Model[ModelType.YAML] to this, maybe with a working dir
            msg.yaml_path = obstacle.model.get([ModelType.YAML]).path

            srv.InteractiveObstacles.append(msg)

        max_num_try = 1
        i_curr_try = 0
        rospy.logdebug("trying to call service with interactive obstacles: ")

        while i_curr_try < max_num_try:
            # try to call service
            response = self._spawn_interactive_obstacles_srv.call(
                srv.InteractiveObstacles)

            if not response.success:  # if service not succeeds, do something and redo service
                # rospy.logwarn(
                #     f"spawn static obstacle failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
                i_curr_try += 1
            else:
                break
        rospy.set_param(
            f'{self._ns_prefix()}agent_topic_string', self.agent_topic_str)
        rospy.set_param("respawn_static", True)
        rospy.set_param("respawn_interactive", True)
        return

    def spawn_dynamic_obstacles(self, obstacles):
        srv = SpawnPeds()
        srv.peds = []

        self.agent_topic_str = ''
        for obstacle in obstacles:
            msg = Ped()
            msg.id = next(self._id_gen)

            msg.pos = Point(*obstacle.position)

            self.agent_topic_str += f',pedsim_agent_{obstacle.name}/0'
            msg.type = obstacle.extra.get("type")
            msg.yaml_file = obstacle.model.get([ModelType.YAML]).path

            msg.type = "adult"
            msg.number_of_peds = 1
            msg.vmax = obstacle.extra.get("vmax", Pedsim.VMAX)
            msg.start_up_mode = obstacle.extra.get(
                "start_up_mode", Pedsim.START_UP_MODE)
            msg.wait_time = obstacle.extra.get("wait_time", Pedsim.WAIT_TIME)
            msg.trigger_zone_radius = obstacle.extra.get(
                "trigger_zone_radius", Pedsim.TRIGGER_ZONE_RADIUS)
            msg.chatting_probability = obstacle.extra.get(
                "chatting_probability", Pedsim.CHATTING_PROBABILITY)
            msg.tell_story_probability = obstacle.extra.get(
                "tell_story_probability", Pedsim.TELL_STORY_PROBABILITY)
            msg.group_talking_probability = obstacle.extra.get(
                "group_talking_probability", Pedsim.GROUP_TALKING_PROBABILITY)
            msg.talking_and_walking_probability = obstacle.extra.get(
                "talking_and_walking_probability", Pedsim.TALKING_AND_WALKING_PROBABILITY)
            msg.requesting_service_probability = obstacle.extra.get(
                "requesting_service_probability", Pedsim.REQUESTING_SERVICE_PROBABILITY)
            msg.requesting_guide_probability = obstacle.extra.get(
                "requesting_guide_probability", Pedsim.REQUESTING_GUIDE_PROBABILITY)
            msg.requesting_follower_probability = obstacle.extra.get(
                "requesting_follower_probability", Pedsim.REQUESTING_FOLLOWER_PROBABILITY)
            msg.max_talking_distance = obstacle.extra.get(
                "max_talking_distance", Pedsim.MAX_TALKING_DISTANCE)
            msg.max_servicing_radius = obstacle.extra.get(
                "max_servicing_radius", Pedsim.MAX_SERVICING_RADIUS)
            msg.talking_base_time = obstacle.extra.get(
                "talking_base_time", Pedsim.TALKING_BASE_TIME)
            msg.tell_story_base_time = obstacle.extra.get(
                "tell_story_base_time", Pedsim.TELL_STORY_BASE_TIME)
            msg.group_talking_base_time = obstacle.extra.get(
                "group_talking_base_time", Pedsim.GROUP_TALKING_BASE_TIME)
            msg.talking_and_walking_base_time = obstacle.extra.get(
                "talking_and_walking_base_time", Pedsim.TALKING_AND_WALKING_BASE_TIME)
            msg.receiving_service_base_time = obstacle.extra.get(
                "receiving_service_base_time", Pedsim.RECEIVING_SERVICE_BASE_TIME)
            msg.requesting_service_base_time = obstacle.extra.get(
                "requesting_service_base_time", Pedsim.REQUESTING_SERVICE_BASE_TIME)
            msg.force_factor_desired = obstacle.extra.get(
                "force_factor_desired", Pedsim.FORCE_FACTOR_DESIRED)
            msg.force_factor_obstacle = obstacle.extra.get(
                "force_factor_obstacle", Pedsim.FORCE_FACTOR_OBSTACLE)
            msg.force_factor_social = obstacle.extra.get(
                "force_factor_social", Pedsim.FORCE_FACTOR_SOCIAL)
            msg.force_factor_robot = obstacle.extra.get(
                "force_factor_robot", Pedsim.FORCE_FACTOR_ROBOT)
            msg.waypoint_mode = obstacle.extra.get(
                "waypoint_mode", Pedsim.WAYPOINT_MODE)  # or 1 check later

            msg.waypoints = []

            for waypoint in obstacle.waypoints:
                p = Point(*waypoint)
                msg.waypoints.append(p)

            srv.peds.append(msg)

        max_num_try = 1
        i_curr_try = 0
        while i_curr_try < max_num_try:
            # try to call service
            response = self._respawn_peds_srv.call(srv.peds)

            if not response.success:  # if service not succeeds, do something and redo service
                # rospy.logwarn(
                #     f"spawn human failed! trying again... [{i_curr_try+1}/{max_num_try} tried]")
                i_curr_try += 1
            else:
                break
        rospy.set_param(
            f'{self._ns_prefix()}agent_topic_string', self.agent_topic_str)
        rospy.set_param("respawn_dynamic", True)

    def spawn_line_obstacle(self, name, _from, _to):
        return

    def remove_obstacles(self):
        self._remove_all_interactive_obstacles_srv.call()
        self._remove_peds_srv.call()

    def _interactive_actor_poses_callback(self, actors):
        if rospy.get_param("respawn_interactive"):
            if ModelType.SDF in self._simulator.MODEL_TYPES:
                for actor in actors.waypoints:
                    if "interactive" in actor.name:
                        actor_name = str(actor.name)
                        orientation = float(re.findall(
                            r'\(.*?\)', str(actor.name))[0].replace("(", "").replace(")", "").replace(",", "."))
                        direction_x = float(actor.name[actor.name.index(
                            "{")+1: actor.name.index("}")].replace(",", "."))
                        direction_y = float(actor.name[actor.name.index(
                            "[")+1: actor.name.index("]")].replace(",", "."))
                        ob_type = actor.name[actor.name.index(
                            "&")+1: actor.name.index("!")]
                        rot = Rotation.from_euler(
                            'xyz', [0, 0, orientation], degrees=False)
                        rot_quat = rot.as_quat()

                        rospy.loginfo(
                            "Spawning interactive: actor_id = %s", actor_name)

                        #TODO get rid of this
                        z = os.path.join(RosPack().get_path('pedsim_gazebo_plugin'), "models", f"{ob_type}.sdf")

                        with open(z) as file_xml:
                            x = file_xml.read()

                        model_pose = Pose(Point(x=actor.position.x-direction_x,
                                                y=actor.position.y-direction_y,
                                                z=actor.position.z),
                                        Quaternion(rot_quat[0], rot_quat[1], rot_quat[2], rot_quat[3]))

                        self._simulator.spawn_obstacle(
                            Obstacle(
                                name=actor_name,
                                position=(model_pose.position.x, model_pose.position.y, model_pose.orientation.z),
                                model=ModelWrapper.from_model(
                                    Model(
                                        type=ModelType.SDF,
                                        name=actor_name,
                                        description=x,
                                        path=""
                                    )
                                ),
                                extra=dict()
                            )
                        )
            rospy.set_param("respawn_interactive", False)

        if rospy.get_param("respawn_static"):

            if ModelType.SDF in self._simulator.MODEL_TYPES:
                for actor in actors.waypoints:
                    if "static" in actor.name:

                        actor_name = str(actor.name)
                        orientation = float(re.findall(
                            r'\(.*?\)', str(actor.name))[0].replace("(", "").replace(")", "").replace(",", "."))
                        direction_x = float(actor.name[actor.name.index(
                            "{")+1: actor.name.index("}")].replace(",", "."))
                        direction_y = float(actor.name[actor.name.index(
                            "[")+1: actor.name.index("]")].replace(",", "."))
                        ob_type = actor.name[actor.name.index(
                            "&")+1: actor.name.index("!")]

                        rot = Rotation.from_euler(
                            'xyz', [0, 0, orientation], degrees=False)
                        rot_quat = rot.as_quat()

                        rospy.loginfo("Spawning static: actor_id = %s", actor_name)

                        #TODO get rid of this
                        with open(os.path.join(RosPack().get_path('pedsim_gazebo_plugin'), "models", "table.sdf")) as file_xml:
                            x = file_xml.read()

                        model_pose = Pose(Point(x=actor.position.x-direction_x,
                                                y=actor.position.y-direction_y,
                                                z=actor.position.z),
                                        Quaternion(rot_quat[0],
                                                    rot_quat[1],
                                                    rot_quat[2],
                                                    rot_quat[3]))

                        self._simulator.spawn_obstacle(
                            Obstacle(
                                name=actor_name,
                                position=(model_pose.position.x, model_pose.position.y, model_pose.orientation.z),
                                model=ModelWrapper.from_model(
                                    Model(
                                        type=ModelType.SDF,
                                        name=actor_name,
                                        description=x,
                                        path=""
                                    )
                                ),
                                extra=dict()
                            )
                        )

            rospy.set_param("respawn_static", False)

    def _dynamic_actor_poses_callback(self, actors):
        if rospy.get_param("respawn_dynamic"):
            if ModelType.SDF in self._simulator.MODEL_TYPES:
                for actor in actors.agent_states:
                    actor_id = str(actor.id)
                    actor_pose = actor.pose
                    rospy.loginfo("Spawning dynamic obstacle: actor_id = %s", actor_id)

                    self._simulator.spawn_obstacle(
                        Obstacle(
                            name=actor_id,
                            position=(actor_pose.position.x, actor_pose.position.y, actor_pose.orientation.z),
                            model=ModelWrapper.from_model(model=Model(type=ModelType.SDF, name=actor_id, description=self._xml_string, path="")),
                            extra=dict()
                        )
                    )
                    
            rospy.set_param("respawn_dynamic", False)
