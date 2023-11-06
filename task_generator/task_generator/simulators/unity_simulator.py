import rospy

from task_generator.simulators.simulator_factory import SimulatorFactory
from task_generator.utils import rosparam_get
from tf.transformations import quaternion_from_euler
from task_generator.constants import Constants
from task_generator.simulators.base_simulator import BaseSimulator

from task_generator.shared import EntityProps, ModelType, Robot, ObstacleProps, PositionOrientation

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
        self._spawn_model = rospy.ServiceProxy(
            self._namespace("unity", "spawn_model"), Empty
        )
        self._remove_model_srv = rospy.ServiceProxy(
            self._namespace("unity", "delete_model"), Empty
        )
        self._move_model_srv = rospy.ServiceProxy(
            self._namespace("unity", "set_model_state"), Empty, persistent=True
        )
        self._goal_pub = rospy.Publisher(
            self._namespace("unity", "set_goal"), PoseStamped, queue_size=1, latch=True
        )

        rospy.loginfo("...Unity services now available.")

    def before_reset_task(self):
        pass

    def after_reset_task(self):
        pass

    def spawn_entity(self, entity: EntityProps) -> bool:
        req = EmptyRequest()
        self._spawn_model(req)

    def spawn_robot(self, robot: Robot) -> str:
        """
        Spawn a robot in the simulator.
        """
        req = EmptyRequest()
        self._spawn_model(req)

    def move_entity(self, name: str, pos: PositionOrientation):
        """
        Move the robot to the given position.
        """
        raise NotImplementedError()

    def delete_entity(self, name: str) -> bool:
        raise NotImplementedError()

    def _publish_goal(self, goal):
        raise NotImplementedError()
