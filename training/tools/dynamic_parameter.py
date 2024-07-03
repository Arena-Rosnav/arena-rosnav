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

        self._sub = node.create_subscription(message_type, 
            f"{self._topic_prefix}{self._key}", self._callback
        )


class DynamicParameterPublisher:
    """
    Represents a dynamic parameter that can be updated through a ROS topic.
    The parameter is updated by publishing to a ROS topic.

    Args:
        key (Any): The key or name of the parameter to be updated.
        message_type: The ROS message type used for updating the parameter.
        topic_prefix (str, optional): The prefix for the ROS topic used for updating the parameter.
            Defaults to "/update_param/".

    Attributes:
        _key (Any): The key or name of the parameter to be updated.
        _topic_prefix (str): The prefix for the ROS topic used for updating the parameter.
        _pub (rospy.Publisher): The ROS publisher for publishing updates to the parameter.
    """

    def __init__(self, key: Any, message_type, topic_prefix: str = "/update_param/"):
        self._key = key
        self._topic_prefix = topic_prefix

        self._pub = node.create_publisher(message_type, queue_size=1
        , 
            f"{self._topic_prefix}{self._key}")

    def __call__(self, value: Any):
        """
        Publishes the value to the parameter.

        Args:
            value (Any): The value to be published.
        """
        self._pub.publish(value)
