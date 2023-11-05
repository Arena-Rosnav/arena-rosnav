import rospy
import os
import requests

from task_generator.simulators.simulator_factory import SimulatorFactory
from task_generator.utils import rosparam_get
from tf.transformations import quaternion_from_euler
from task_generator.constants import Constants
from task_generator.simulators.base_simulator import BaseSimulator

from task_generator.shared import EntityProps, ModelType, Robot, ObstacleProps, PositionOrientation

from std_msgs.msg import Empty
from std_srvs.srv import Empty

T = Constants.WAIT_FOR_SERVICE_TIMEOUT


@SimulatorFactory.register(Constants.Simulator.UNITY)
class UnitySimulator(BaseSimulator):

    _robot_name: str

    def __init__(self, namespace: str):
        super().__init__(namespace)
        self._robot_name = rosparam_get(str, "robot_model", "")

        rospy.loginfo("Waiting for Unity services...")

        rospy.wait_for_service("unity/spawn_model", timeout=T)

        self._spawn_model[ModelType.URDF] = rospy.ServiceProxy(
            "unity/spawn_model", Empty
        )
        self._spawn_model[ModelType.SDF] = rospy.ServiceProxy(
            "unity/spawn_model", Empty
        )

        rospy.loginfo("...Unity services now available.")

    def before_reset_task(self):
        """
        Is executed each time before the task is reseted. This is useful in
        order to pause the simulation and physics.
        """
        raise NotImplementedError()

    def after_reset_task(self):
        """
        Is executed after the task is reseted. This is useful to unpause the
        simulation and physics.
        """
        raise NotImplementedError()

    # OBSTACLE
    def spawn_obstacle(self, obstacle: ObstacleProps) -> bool:
        raise NotImplementedError()
    
    def spawn_entity(self, entity: EntityProps) -> bool:
        self._spawn_model("This is only an example call of the function")
        raise NotImplementedError()
        
    # ROBOT
    def spawn_robot(self, robot: Robot) -> str:
        """
        Spawn a robot in the simulator.
        """
        raise NotImplementedError()

    def move_entity(self, name: str, pos: PositionOrientation):
        """
        Move the robot to the given position.
        """
        raise NotImplementedError()

    def delete_entity(self, name: str) -> bool:
        raise NotImplementedError()
