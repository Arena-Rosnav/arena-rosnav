from time import sleep
from typing import Any, Dict

import numpy as np

import rospy
from cv_bridge import CvBridge
from unity_msgs.msg import Collision
from sensor_msgs.msg import Image
from task_generator.shared import Namespace

from ..constants import OBS_DICT_KEYS, TOPICS, MAX_WAIT, SLEEP
from ..utils import false_params
from .collector_unit import CollectorUnit


class RgbdCollectorUnit(CollectorUnit):
    """
    A class for collecting rgbd observations (depth and color).

    Attributes:
        _image_depth (np.ndarray): Received depth observation data.
        _image_color (np.ndarray): Received color observation data.

        _image_depth_sub (rospy.Subscriber): Subscriber for depth data.
        _image_color_sub (rospy.Subscriber): Subscriber for color data.

        _received_image_depth (bool): Flag indicating if depth data has been received.
        _received_image_depth (bool): Flag indicating if color data has been received.

        _first_reset (bool): Flag indicating if it is the first reset.
        
        _image_bridge (CvBridge): Bridge to convert incoming data objects into numpy arrays.
    """

    # Retrieved information
    _image_depth: np.ndarray
    _image_color: np.ndarray

    # Subscriptions
    _image_depth_sub: rospy.Subscriber
    _image_color_sub: rospy.Subscriber

    # Received Flags
    _received_image_depth: bool
    _received_image_color: bool

    _first_reset: bool
    
    _image_bridge: CvBridge

    def __init__(self, ns: Namespace, observation_manager, *args, **kwargs) -> None:
        """
        Initialize the RgbdCollectorUnit.

        Args:
            ns (Namespace): Namespace for the collector unit.
            observation_manager: Observation manager holding this collector unit.
        """
        super().__init__(ns, observation_manager)
                
        self._image_depth = None
        self._image_color = None

        self._image_depth_sub: rospy.Subscriber = None
        self._image_color_sub: rospy.Subscriber = None

        self._received_image_color = False
        self._received_image_depth = False

        self._first_reset = True

        self._image_bridge = CvBridge()

    def init_subs(self):
        """
        Initialize the subscribers for sensor data.
        """
        self._image_color_sub = rospy.Subscriber(
            self._ns(TOPICS.IMAGE_COLOR),
            Image,
            self._cb_image_color,
            tcp_nodelay=True
        )
        self._image_depth_sub = rospy.Subscriber(
            self._ns(TOPICS.IMAGE_DEPTH),
            Image,
            self._cb_image_depth,
            tcp_nodelay=True
        )

    def wait(self):
        """
        Wait for the required data to be received.
        """
        if self._first_reset:
            self._first_reset = False
            return

        for _ in range(int(MAX_WAIT / SLEEP)):
            if self._received_image_depth and self._received_image_color:
                return

            sleep(SLEEP)

        raise TimeoutError(
            f"""Couldn't retrieve data for: {false_params(
                    image_color=self._received_image_color,
                    image_depth=self._received_image_depth
                )}"""
        )

    def get_observations(
        self, obs_dict: Dict[str, Any], *args, **kwargs
    ) -> Dict[str, Any]:
        """
        Get the observations from the collected data.

        Args:
            obs_dict (Dict[str, Any]): Dictionary to store the observations.

        Returns:
            Dict[str, Any]: Updated dictionary with observations.
        """
        obs_dict = super().get_observations(obs_dict)

        obs_dict.update(
            {
                OBS_DICT_KEYS.IMAGE_COLOR: self._image_color,
                OBS_DICT_KEYS.IMAGE_DEPTH: self._image_depth
            }
        )
        
        self._received_image_color = False
        self._received_image_depth = False

        return obs_dict

    def _cb_image_color(self, image_msg: Image):
        """
        Callback function for receiving rgb data. Converts image msg to cv Mat to np ndarray.

        Args:
            image_msg (Image): Received rgb image message.
        """
        cv_mat = self._image_bridge.imgmsg_to_cv2(image_msg)
        # let the channel dim be the first dim and remove a-channel
        self._image_color = (np.asarray(cv_mat, dtype=np.float32)[:,:,0:3]).transpose((2, 0, 1))
        self._received_image_color = True

    def _cb_image_depth(self, image_msg: Image):
        """Callback function for receiving depth data. Converts image msg to cv Mat to np ndarray.

        Args:
            image_msg (Image): Received depth image message.
        """
        cv_mat = self._image_bridge.imgmsg_to_cv2(image_msg)
        self._image_depth = np.asarray(cv_mat, dtype=np.float32)
        self._received_image_depth = True
