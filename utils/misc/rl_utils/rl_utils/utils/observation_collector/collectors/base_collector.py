from abc import ABC, abstractmethod
from typing import Type, TypeVar

import rospy
import numpy as np

T = TypeVar("T")
D = TypeVar("D")


class BaseUnit(ABC):
    name: str

    def __init__(self, *args, **kwargs) -> None:
        pass

    def __repr__(self):
        return f"{self.name}"


class ObservationCollectorUnit(BaseUnit, ABC):
    """
    Base class for collector units.

    Attributes:
        name (str): The name of the collector unit.
        topic (str): The topic associated with the collector unit.
        msg_data_class (Type[T]): The type of the message data expected by the collector unit.
        data_class (Type[D]): The type of the preprocessed data returned by the collector unit.
        is_topic_agent_specific (bool): Indicates whether the topic is agent-specific. If yes, the agent ns is appended to the topic.

    Methods:
        preprocess(msg: T) -> D:
            Preprocesses the message data and returns the preprocessed data.

    """

    name: str = ""
    topic: str = ""
    msg_data_class: Type[T] = T
    data_class: Type[D] = D

    is_topic_agent_specific: bool = True

    @abstractmethod
    def preprocess(self, msg: T) -> D:
        """
        Preprocesses the message data and returns the preprocessed data.

        Args:
            msg (T): The message data to be preprocessed.

        Returns:
            D: The preprocessed data.

        """
        if self.data_class and not isinstance(msg, self.msg_data_class):
            rospy.logwarn_once(
                f"[{self.__class__.__name__}] Expected {self.msg_data_class} but got {type(msg)}"
            )
