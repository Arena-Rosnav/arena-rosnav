import rospy

import gazebo_msgs.msg as gazebo_msgs
import gazebo_msgs.srv as gazebo_srvs

import geometry_msgs.msg as geometry_msgs

import std_srvs.srv as std_srvs


from task_generator.simulators.simulator_factory import SimulatorFactory
from task_generator.utils import rosparam_get, Utils
from task_generator.constants import Config, Constants
from task_generator.simulators.base_simulator import BaseSimulator
from task_generator.simulators.simulator_factory import SimulatorFactory

from task_generator.shared import ModelType, Namespace, PositionOrientation, RobotProps


T = Config.General.WAIT_FOR_SERVICE_TIMEOUT


@SimulatorFactory.register(Constants.Simulator.GAZEBO)
class GazeboSimulator(BaseSimulator):

    _goal_pub: rospy.Publisher
    _robot_name: str

    _unpause: rospy.ServiceProxy
    _pause: rospy.ServiceProxy
    _remove_model_srv: rospy.ServiceProxy

    def __init__(self, namespace):

        super().__init__(namespace)
        self._goal_pub = rospy.Publisher(
            self._namespace("/goal"),
            geometry_msgs.PoseStamped,
            queue_size=1,
            latch=True
        )
        self._robot_name = rosparam_get(str, "robot_model", "")

        rospy.wait_for_service("/gazebo/spawn_urdf_model")
        rospy.wait_for_service("/gazebo/spawn_sdf_model")
        rospy.wait_for_service("/gazebo/set_model_state")
        rospy.wait_for_service("/gazebo/set_model_state", timeout=20)

        self._spawn_model[ModelType.URDF] = rospy.ServiceProxy(
            self._namespace(
                "gazebo", "spawn_urdf_model"), gazebo_srvs.SpawnModel
        )
        self._spawn_model[ModelType.SDF] = rospy.ServiceProxy(
            self._namespace(
                "gazebo", "spawn_sdf_model"), gazebo_srvs.SpawnModel
        )
        self._move_model_srv = rospy.ServiceProxy(
            "/gazebo/set_model_state", gazebo_srvs.SetModelState, persistent=True
        )
        self._unpause = rospy.ServiceProxy(
            "/gazebo/unpause_physics", std_srvs.Empty)
        self._pause = rospy.ServiceProxy(
            "/gazebo/pause_physics", std_srvs.Empty)

        rospy.loginfo("Waiting for gazebo services...")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        rospy.wait_for_service("gazebo/delete_model")

        rospy.loginfo("service: spawn_sdf_model is available ....")
        self._remove_model_srv = rospy.ServiceProxy(
            "gazebo/delete_model", gazebo_srvs.DeleteModel)

    def before_reset_task(self):
        self._pause()

    def after_reset_task(self):
        try:
            self._unpause()
        except rospy.service.ServiceException as e:  # gazebo isn't the most reliable
            rospy.logwarn(e)

    # ROBOT

    def move_entity(self, name, position: PositionOrientation):

        request = gazebo_srvs.SetModelStateRequest()
        request.model_state = gazebo_msgs.ModelState()

        request.model_state.model_name = name
        request.model_state.pose = Utils.pos_to_pose(position)
        request.model_state.reference_frame = "world"

        return bool(self._move_model_srv(request).success)

    def spawn_entity(self, entity):
        request = gazebo_srvs.SpawnModelRequest()

        model = entity.model.get(self.MODEL_TYPES)

        request.model_name = entity.name
        request.model_xml = model.description
        request.initial_pose = Utils.pos_to_pose(entity.position)
        request.robot_namespace = Namespace(entity.name)
        request.reference_frame = "world"

        if isinstance(entity, RobotProps):
            rospy.set_param(request.robot_namespace(
                "robot_description"), model.description)
            rospy.set_param(request.robot_namespace(
                "tf_prefix"), str(request.robot_namespace))

        res = self.spawn_model(model.type, request)
        return bool(res.success)

    def delete_entity(self, name):
        res: gazebo_srvs.DeleteModelResponse = self._remove_model_srv(
            gazebo_srvs.DeleteModelRequest(model_name=name))
        return bool(res.success)

    def _publish_goal(self, goal: PositionOrientation):
        goal_msg = geometry_msgs.PoseStamped()
        goal_msg.header.seq = 0
        goal_msg.header.stamp = rospy.get_rostime()
        goal_msg.header.frame_id = "map"
        goal_msg.pose = Utils.pos_to_pose(goal)

        self._goal_pub.publish(goal_msg)