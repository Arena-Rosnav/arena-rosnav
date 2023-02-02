import rospy
import os
import rospkg
import subprocess
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from pedsim_srvs.srv import SpawnPeds
from std_msgs.msg import Empty
from std_srvs.srv import Empty, SetBool, Trigger
from task_generator.environments.environment_factory import EnvironmentFactory
from tf.transformations import quaternion_from_euler

from ..constants import Constants
from .base_environment import BaseEnvironment
from .environment_factory import EnvironmentFactory

T = Constants.WAIT_FOR_SERVICE_TIMEOUT


@EnvironmentFactory.register("gazebo")
class GazeboEnvironment(BaseEnvironment):
    def __init__(self, namespace):
        super().__init__(namespace)

        self._goal_pub = rospy.Publisher(self._ns_prefix("/goal"), PoseStamped, queue_size=1, latch=True)

        self._robot_name = rospy.get_param("robot_model")

        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")
        # rospy.wait_for_service("/pedsim_simulator/spawn_peds", timeout=T)
        # rospy.wait_for_service("/pedsim_simulator/reset_all_peds", timeout=T)
        # rospy.wait_for_service("/pedsim_simulator/remove_all_peds", timeout=T)

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
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)

        self.map_manager = None

    def before_reset_task(self):
        self.pause()
        pass

    def after_reset_task(self):
        self.unpause()
        pass

    def remove_all_obstacles(self):
        # self._remove_peds_srv(True)
        pass

    def spawn_pedsim_agents(self, agents):
        peds = [agent.getPedMsg() for agent in agents]
        # self._spawn_peds_srv(peds)

    def reset_pedsim_agents(self):
        # self._reset_peds_srv()
        pass

    def spawn_obstacle(self, position, yaml_path=""):
        pass

    def spawn_random_dynamic_obstacle(self, **args):
        # s_pos = args["position"]
        # w_pos = self.map_manager.get_random_pos_on_map(
        #     safe_dist=1, forbidden_zones=[s_pos]
        # )
        # ped = self._create_simple_ped([1], [s_pos], [w_pos])
        # agents = [PedsimAgent.fromDict(a) for a in ped]
        # self.spawn_pedsim_agents(agents)

        pass

    def spawn_random_static_obstacle(self, **args):
        pass

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
        pose.orientation = Quaternion(
            *quaternion_from_euler(0.0, 0.0, pos[2], axes="sxyz")
        )
        model_state_request.pose = pose
        model_state_request.reference_frame = "world"

        self._move_model_srv(model_state_request)

    def spawn_robot(self, name, robot_name, namespace_appendix=""):
        request = SpawnModelRequest()

        robot_namespace = self._ns_prefix(namespace_appendix)
        
        robot_description = GazeboEnvironment.get_robot_description(
            robot_name, robot_namespace
        )
        rospy.set_param(os.path.join(robot_namespace, "robot_description"), robot_description)
        rospy.set_param(os.path.join(robot_namespace, "tf_prefix"), robot_namespace)
        
        request.model_name = name
        request.model_xml = robot_description
        request.robot_namespace = robot_namespace
        request.reference_frame = "world"

        self._spawn_model_srv(request)

    ## HELPER FUNCTIONS
    def _create_simple_ped(self, ids, s_pos, w_pos):
        # creates pedsim-agents
        peds = []
        for id, spos, wpos in zip(ids, s_pos, w_pos):
            peds.append(
                {
                    "name": "Pedestrian",
                    "id": id,
                    "pos": [*spos],
                    "type": "adult",
                    "yaml_file": "person_two_legged.model.yaml",
                    "number_of_peds": 1,
                    "vmax": 0.3,
                    "start_up_mode": "default",
                    "wait_time": 0.0,
                    "trigger_zone_radius": 0.0,
                    "chatting_probability": 0.01,
                    "tell_story_probability": 0,
                    "group_talking_probability": 0.01,
                    "talking_and_walking_probability": 0.01,
                    "requesting_service_probability": 0.01,
                    "requesting_guide_probability": 0.01,
                    "requesting_follower_probability": 0.01,
                    "max_talking_distance": 5,
                    "max_servicing_radius": 5,
                    "talking_base_time": 10,
                    "tell_story_base_time": 0,
                    "group_talking_base_time": 10,
                    "talking_and_walking_base_time": 6,
                    "receiving_service_base_time": 20,
                    "requesting_service_base_time": 30,
                    "force_factor_desired": 1,
                    "force_factor_obstacle": 1,
                    "force_factor_social": 5,
                    "force_factor_robot": 1,
                    "waypoints": [[*spos], [*wpos]],
                    "waypoint_mode": 0,
                }
            )
        return peds

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