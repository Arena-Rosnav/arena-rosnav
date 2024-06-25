from typing import Type

import unity_msgs.msg as unity_msgs

from .base_collector import ObservationCollectorUnit

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
    msg_data_class: Type[unity_msgs.Collision] = unity_msgs.Collision
    data_class: Type[bool] = bool
    up_to_date_required: bool = True

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
