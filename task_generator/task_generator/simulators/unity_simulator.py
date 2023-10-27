import rospy
import os
import subprocess
import requests
import time

from task_generator.simulators.simulator_factory import SimulatorFactory
from task_generator.utils import rosparam_get
from tf.transformations import quaternion_from_euler
from task_generator.constants import Constants
from task_generator.simulators.base_simulator import BaseSimulator

from task_generator.shared import ModelType, Robot, ObstacleProps, PositionOrientation


T = Constants.WAIT_FOR_SERVICE_TIMEOUT


@SimulatorFactory.register(Constants.Simulator.UNITY)
class UnitySimulator(BaseSimulator):

    _robot_name: str

    def __init__(self, namespace: str):
        super().__init__(namespace)
        self._robot_name = rosparam_get(str, "robot_model", "")
        self._unity_file = "~/arena_ws/src/arena-rosnav/utils/misc/unity_simulator/unity_simulator.x86_64"

        os.system(self._unity_file + "&")


        # self._spawn_model[ModelType.URDF] = # TODO: Insert function to spawn model
        # self._spawn_model[ModelType.SDF] = # TODO: Insert function to spawn model

        rospy.loginfo("Waiting for Unity services...")

        # Wait until the RESTful Server of Unity is online
        while requests.get('http://127.0.0.1:8080/init', timeout=T).text != "Connected":
            continue
        rospy.loginfo("Unity Simulator available")

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
