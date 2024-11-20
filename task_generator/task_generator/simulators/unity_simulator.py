from numpy import full
import rospy

from task_generator.shared import Namespace, rosparam_get
from task_generator.manager.utils import WorldWalls
from tf.transformations import quaternion_from_euler
from task_generator.constants import Constants, UnityConstants
from task_generator.constants.runtime import Configuration
from task_generator.simulators import BaseSimulator

from task_generator.shared import ModelType, Robot, Obstacle

# Message Types
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, DeleteModel, SpawnModel, SpawnModelRequest, DeleteModelRequest, DeleteModelResponse
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from unity_msgs.srv import SpawnWalls, SpawnWallsRequest
from unity_msgs.msg import Wall

T = Configuration.General.WAIT_FOR_SERVICE_TIMEOUT


class UnitySimulator(BaseSimulator):

    _robot_name: str

    def __init__(self, namespace: Namespace):
        super().__init__(namespace)

        self._robot_name = rosparam_get(str, "robot_model", "")

        rospy.loginfo(
            "[Unity Simulator ns:" +
            self._namespace +
            "] Waiting for Unity services...")

        rospy.loginfo(
            "[Unity Simulator ns:" +
            self._namespace +
            "] namespace:" +
            self._namespace("unity") +
            ", robot name: " +
            self._robot_name)

        rospy.wait_for_service(self._namespace(
            "unity", "spawn_walls"), timeout=T)
        rospy.wait_for_service(self._namespace(
            "unity", "spawn_model"), timeout=T)
        rospy.wait_for_service(self._namespace(
            "unity", "delete_model"), timeout=T)
        rospy.wait_for_service(self._namespace(
            "unity", "set_model_state"), timeout=T)

        rospy.loginfo(
            "[Unity Simulator ns:" +
            self._namespace +
            "] found all unity services")

        # TODO: Custom Message Types
        self._spawn_walls_srv = rospy.ServiceProxy(
            self._namespace("unity", "spawn_walls"), SpawnWalls
        )
        self._spawn_model[ModelType.SDF] = rospy.ServiceProxy(
            self._namespace("unity", "spawn_model"), SpawnModel
        )
        self._spawn_model[ModelType.URDF] = rospy.ServiceProxy(
            self._namespace("unity", "spawn_model"), SpawnModel
        )
        self._remove_model_srv = rospy.ServiceProxy(
            self._namespace("unity", "delete_model"), DeleteModel
        )
        self._move_model_srv = rospy.ServiceProxy(
            self._namespace("unity", "set_model_state"), SetModelState, persistent=True
        )
        self._goal_pub = rospy.Publisher(
            self._namespace("unity", "set_goal"), PoseStamped, queue_size=1, latch=True
        )

        rospy.loginfo("...Unity services now available.")

    def before_reset_task(self):
        pass

    def after_reset_task(self):
        pass

    def spawn_entity(self, entity):
        request = SpawnModelRequest()

        model = entity.model.get(self.MODEL_TYPES)
        # rospy.loginfo("[Unity Simulator ns:" + self._namespace + "] Spawn Request for " + entity.name)

        request.model_name = model.name
        request.model_xml = model.description
        request.robot_namespace = entity.name
        request.reference_frame = "world"

        # send coordinates in the normal ROS refrence frame (FLU)
        request.initial_pose = Pose(
            position=Point(
                x=entity.position[0],
                y=entity.position[1],
                z=0.35
            ),
            orientation=Quaternion(
                *quaternion_from_euler(0.0, 0.0, entity.position[2], axes="sxyz"))
        )

        if isinstance(entity, Robot):
            full_robot_ns = self._namespace(entity.name)
            rospy.set_param(full_robot_ns(
                "robot_description"), model.description)
            rospy.set_param(full_robot_ns(
                "tf_prefix"), str(request.robot_namespace))

        if isinstance(entity, Obstacle) and "<actor" not in model.description:
            request.model_xml = model.name

        res = self.spawn_model(model.type, request)
        return res.success

    def move_entity(self, name, position):
        # rospy.loginfo("[Unity Simulator ns:" + self._namespace + "] Move Request for " + name)

        request = SetModelStateRequest()
        request.model_state = ModelState()

        request.model_state.model_name = name
        pose = Pose(
            position=Point(
                x=position[0],
                y=position[1],
                z=0.35
            ),
            orientation=Quaternion(
                *quaternion_from_euler(0.0, 0.0, position[2], axes="sxyz")
            )
        )
        request.model_state.pose = pose
        request.model_state.reference_frame = "world"

        self._move_model_srv(request)

    def delete_entity(self, name):
        # rospy.loginfo("[Unity Simulator ns:" + self._namespace + "] Delete Request for " + name)
        res: DeleteModelResponse = self._remove_model_srv(
            DeleteModelRequest(model_name=name))
        return bool(res.success)

    def _publish_goal(self, goal):
        # rospy.loginfo("[Unity Simulator ns:" + self._namespace + "] Goal Request")

        goal_msg = PoseStamped()
        goal_msg.header.seq = 0
        goal_msg.header.stamp = rospy.get_rostime()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal[0]
        goal_msg.pose.position.z = goal[1]

        goal_msg.pose.orientation.w = 0
        goal_msg.pose.orientation.x = 0
        goal_msg.pose.orientation.y = 0
        goal_msg.pose.orientation.z = 1

        self._goal_pub.publish(goal_msg)

    def spawn_walls(self, walls: WorldWalls):
        # rospy.loginfo("[Unity Simulator ns:" + self._namespace + "] Spawn Wall Request")

        # send a spawn request to unity for all walls
        request = SpawnWallsRequest()
        request.walls = []
        for wall in walls:
            wall_req = Wall(
                start=Point(x=wall[0].x, y=wall[0].y, z=0),
                end=Point(x=wall[1].x, y=wall[1].y,
                          z=UnityConstants.WALL_HEIGHT)
            )
            request.walls.append(wall_req)
        self._spawn_walls_srv(request)
        # rospy.loginfo("[Unity Simulator ns:" + self._namespace + "] Sent Spawn Wall Request")
