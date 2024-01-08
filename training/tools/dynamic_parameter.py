from typing import Any

import rospy


class DynamicParameter:
    """
    Represents a dynamic parameter that can be updated through a ROS topic.
    The parameter is updated by subscribing to a ROS topic and setting the parameter value.

    Args:
        cls (object): The class instance that contains the parameter to be updated.
        key (Any): The key or name of the parameter to be updated.
        message_type: The ROS message type used for updating the parameter.
        topic_prefix (str, optional): The prefix for the ROS topic used for updating the parameter.
            Defaults to "/update_param/".

    Raises:
        AttributeError: If the class instance does not have the specified parameter.

    Attributes:
        _key (Any): The key or name of the parameter to be updated.
        _topic_prefix (str): The prefix for the ROS topic used for updating the parameter.
        _callback (function): The callback function for handling the updated parameter value.
        _sub (rospy.Subscriber): The ROS subscriber for receiving updates to the parameter.
    """

    def __init__(
        self, key: Any, cls: object, message_type, topic_prefix: str = "/update_param/"
    ):
        self._key = key
        self._topic_prefix = topic_prefix

        # check if cls has attribute key
        if not hasattr(cls, f"{key}"):
            raise AttributeError(f"{cls} has no attribute {key}!")

        self._callback = lambda msg: setattr(cls, f"{key}", msg.data)

        self._sub = rospy.Subscriber(
            f"{self._topic_prefix}{self._key}", message_type, self._callback
        )
