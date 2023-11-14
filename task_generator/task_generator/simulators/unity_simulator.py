import rospy

from task_generator.simulators.simulator_factory import SimulatorFactory
from task_generator.utils import rosparam_get
from tf.transformations import quaternion_from_euler
from task_generator.constants import Constants
from task_generator.simulators.base_simulator import BaseSimulator

from task_generator.shared import EntityProps, ModelType, Robot, ObstacleProps, PositionOrientation

# Message Types
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, DeleteModel, SpawnModel, SpawnModelRequest, DeleteModelRequest, DeleteModelResponse
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Empty
from std_srvs.srv import Empty, EmptyRequest

T = Constants.WAIT_FOR_SERVICE_TIMEOUT


@SimulatorFactory.register(Constants.Simulator.UNITY)
class UnitySimulator(BaseSimulator):

    _robot_name: str

    def __init__(self, namespace: str):
        super().__init__(namespace)
        self._robot_name = rosparam_get(str, "robot_model", "")

        rospy.loginfo("Waiting for Unity services...")

        rospy.wait_for_service(self._namespace(
            "unity", "spawn_model"), timeout=T)
        rospy.wait_for_service(self._namespace(
            "unity", "delete_model"), timeout=T)
        rospy.wait_for_service(self._namespace(
            "unity", "set_model_state"), timeout=T)

        # TODO: Proper Message Types
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
        rospy.loginfo("[Unity Simulator] Spawn Request for " + entity.name)
        request = SpawnModelRequest()

        model = entity.model.get(self.MODEL_TYPES)

        request.model_name = entity.name
        request.model_xml = model.description
        request.robot_namespace = self._namespace(entity.name)
        request.reference_frame = "world"
        # Keep in mind that y axis is up
        request.initial_pose = Pose(
            position=Point(
                x=entity.position[0],
                y=0,
                z=entity.position[1]
            ),
            orientation=Quaternion(*quaternion_from_euler(0.0, entity.position[2], 0.0, axes="sxyz")
                                   )
        )

        rospy.set_param(request.robot_namespace(
            "robot_description"), model.description)
        rospy.set_param(request.robot_namespace(
            "tf_prefix"), str(request.robot_namespace))

        res = self.spawn_model(model.type, request)
        return res.success

    def move_entity(self, name, pos):
        rospy.loginfo("[Unity Simulator] Move Request for " + name)

        request = SetModelStateRequest()
        request.model_state = ModelState()

        request.model_state.model_name = name
        pose = Pose()
        # Keep in mind that y axis is up
        pose.position.x = pos[0]
        pose.position.y = 0.35
        pose.position.z = pos[1]
        pose.orientation = Quaternion(
            *quaternion_from_euler(0.0, pos[2], 0.0, axes="sxyz")
        )
        request.model_state.pose = pose
        request.model_state.reference_frame = "world"

        self._move_model_srv(request)

    def delete_entity(self, name):
        rospy.loginfo("[Unity Simulator] Delete Request for " + name)
        res: DeleteModelResponse = self._remove_model_srv(
            DeleteModelRequest(model_name=name))
        return bool(res.success)

    def _publish_goal(self, goal):
        rospy.loginfo("[Unity Simulator] Goal Request")

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
