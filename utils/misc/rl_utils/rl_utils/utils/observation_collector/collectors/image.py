from abc import ABC
from typing import Type

import numpy as np
import sensor_msgs.msg as sensor_msgs
from cv_bridge import CvBridge

from .base_collector import ObservationCollectorUnit

__all__ = ["ImageColorCollector", "ImageDepthCollector"]


class ImageCollector(ObservationCollectorUnit, ABC):
    """
    A class for collecting image observations.

    Attributes:
        name (str): The name of the image collector.
        topic (str): The topic to subscribe to for image messages.
        msg_data_class (Type[sensor_msgs.Image]): The ROS message data class for image messages.
        data_class (Type[np.ndarray]): The data class for storing image data.

    Methods:
        __init__(*args, **kwargs): Initializes the ImageCollector object.
    """

    name: str
    topic: str
    msg_data_class: Type[sensor_msgs.Image] = sensor_msgs.Image
    data_class: Type[np.ndarray] = np.ndarray

    def __init__(self, *args, **kwargs) -> None:
        """
        Initializes the ImageCollector object.

        Args:
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.
        """
        self._image_bridge = CvBridge()


class ImageColorCollector(ImageCollector):
    """
    A class that collects color images as observations.

    Attributes:
        name (str): The name of the collector.
        topic (str): The topic to subscribe to for image messages.
        msg_data_class (Type[sensor_msgs.Image]): The message data class for image messages.
        data_class (Type[np.ndarray]): The data class for the collected images.

    Methods:
        __init__(*args, **kwargs): Initializes the ImageColorCollector object.
        preprocess(msg: sensor_msgs.Image) -> np.ndarray: Preprocesses the image message and returns the processed image.
    """

    name: str = "image_color"
    topic: str = "rgbd/image"

    def preprocess(self, msg: sensor_msgs.Image) -> np.ndarray:
        """
        Preprocesses the image message and returns the processed image.

        Args:
            msg (sensor_msgs.Image): The image message to preprocess.

        Returns:
            np.ndarray: The processed image.
        """
        if msg.data == b'':
            # calling image bridge with empty message will crash program
            return None
        cv_mat = self._image_bridge.imgmsg_to_cv2(msg)
        # let the channel dim be the first dim and remove a-channel
        return (np.asarray(cv_mat, dtype=np.float32)[:, :, 0:3]).transpose((2, 0, 1))


class ImageDepthCollector(ImageCollector):
    """
    A class that collects depth images as observations.

    Attributes:
        name (str): The name of the collector.
        topic (str): The topic to subscribe to for depth images.
        msg_data_class (Type[sensor_msgs.Image]): The ROS message data class for depth images.
        data_class (Type[np.ndarray]): The data class for depth images.

    Methods:
        __init__(*args, **kwargs): Initializes the ImageDepthCollector object.
        preprocess(msg: sensor_msgs.Image) -> np.ndarray: Preprocesses the depth image message.

    """

    name: str = "image_depth"
    topic: str = "rgbd/depth"

    def preprocess(self, msg: sensor_msgs.Image) -> np.ndarray:
        """
        Preprocesses the depth image message.

        Args:
            msg (sensor_msgs.Image): The depth image message.

        Returns:
            np.ndarray: The preprocessed depth image.

        """
        if msg.data == b'':
            # calling image bridge with empty message will crash program
            return None
        cv_mat = self._image_bridge.imgmsg_to_cv2(msg)
        return np.asarray(cv_mat, dtype=np.float32)
