from typing import Type

import rospy
import unity_msgs.msg as unity_msgs
from task_generator.constants import Config, UnityConstants, Constants

from .base_collector import ObservationCollectorUnit
from rl_utils.topic import Namespace
from rl_utils.state_container import SimulationStateContainer

__all__ = ["CollisionCollector", "PedSafeDistCollector", "ObsSafeDistCollector"]


class CollisionCollector(ObservationCollectorUnit):
    """
    A class that collects collision observations from a Unity environment.

    Attributes:
        name (str): The name of the collision collector.
        topic (str): The topic to subscribe to for collision messages.
        msg_data_class (Type[unity_msgs.Collision]): The data class for collision messages.
        data_class (Type[bool]): The data class for the collected collision observations.
        up_to_date_required (bool): Specifies whether value should be kept up to date, i.e. a new message is required for every step.
    """

    name: str = "collision"
    topic: str = "collision"
    msg_data_class: Type[unity_msgs.Collision] = unity_msgs.Collision
    data_class: Type[bool] = bool
    up_to_date_required: bool = True

    def preprocess(self, msg: unity_msgs.Collision) -> bool:
        """
        Preprocesses the collision message.

        Args:
            msg (unity_msgs.Collision): The collision message to preprocess.

        Returns:
            bool: The preprocessed collision observation.
        """
        return msg.collision


class PedSafeDistCollector(ObservationCollectorUnit):
    """
    A class that collects pedestrian safe distance observations.

    Attributes:
        name (str): The name of the collector.
        topic (str): The topic to subscribe to for receiving collision messages.
        msg_data_class (Type[unity_msgs.Collision]): The data class for collision messages.
        data_class (Type[bool]): The data class for the collected observations.
        up_to_date_required (bool): Specifies whether value should be kept up to date, i.e. a new message is required for every step.
    """

    name: str = "ped_safe_dist"
    topic: str = "ped_safe_dist"
    applicable_simulators: list = [Constants.Simulator.UNITY]
    msg_data_class: Type[unity_msgs.Collision] = unity_msgs.Collision
    data_class: Type[bool] = bool
    up_to_date_required: bool = True

    def __init__(
        self,
        ns: Namespace,
        simulation_state_container: SimulationStateContainer,
        *args,
        **kwargs,
    ):
        try:
            super().__init__(*args, **kwargs)
        except RuntimeError as e:
            rospy.logwarn(f"{e}. Collector '{self.name}' will not be used.")
            return

        # Send request to Unity to attach sensor
        service_topic = ns.simulation_ns(
            "unity", UnityConstants.ATTACH_SAFE_DIST_SENSOR_TOPIC
        )

        rospy.wait_for_service(
            service_topic, timeout=Config.General.WAIT_FOR_SERVICE_TIMEOUT
        )
        request = unity_msgs.AttachSafeDistSensorRequest(
            robot_name=ns.robot_ns,
            safe_dist_topic=PedSafeDistCollector.topic,
            ped_safe_dist=True,
            obs_safe_dist=False,
            safe_dist=simulation_state_container.robot.safety_distance,
        )
        response = rospy.ServiceProxy(service_topic, unity_msgs.AttachSafeDistSensor)(
            request
        )
        # Check success
        if not response.success:
            raise rospy.ServiceException(response.message)

    def preprocess(self, msg: unity_msgs.Collision) -> bool:
        """
        Preprocesses the received collision message.

        Args:
            msg (unity_msgs.Collision): The collision message to preprocess.

        Returns:
            bool: The preprocessed observation value.
        """
        return msg.collision


class ObsSafeDistCollector(ObservationCollectorUnit):
    """
    A class that collects safe distance observations from Unity collision messages.

    Attributes:
        name (str): The name of the observation collector unit.
        topic (str): The topic to subscribe to for collision messages.
        msg_data_class (Type[unity_msgs.Collision]): The data class for collision messages.
        data_class (Type[bool]): The data class for safe distance observations.
        up_to_date_required (bool): Specifies whether value should be kept up to date, i.e. a new message is required for every step.
    """

    name: str = "obs_safe_dist"
    topic: str = "obs_safe_dist"
    msg_data_class: Type[unity_msgs.Collision] = unity_msgs.Collision
    data_class: Type[bool] = bool
    up_to_date_required: bool = True

    def preprocess(self, msg: unity_msgs.Collision) -> bool:
        """
        Preprocesses the collision message and returns a boolean value.

        Args:
            msg (unity_msgs.Collision): The collision message to preprocess.

        Returns:
            bool: The preprocessed boolean value.
        """
        return msg.collision
