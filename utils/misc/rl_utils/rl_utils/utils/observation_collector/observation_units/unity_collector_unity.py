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


class UnityCollectorUnit(CollectorUnit):
    """
    A class for collecting Arena Unity specific navigation information such as collider data (safe 
    distances and collisons).

    Attributes:
        _collision (bool): True if robot is currently colliding with something.
        _ped_safe_dist: True if robot is currently within safe dist of a pedestrian.
        _obs_safe_dist: True if robot is currently within safe dist of obstacle.

        _collision_sub (rospy.Subscriber): Subscriber for collision state data.
        _ped_safe_dist_sub (rospy.Subscriber): Subscriber for pedestrian safe dist state data.
        _obs_safe_dist_sub (rospy.Subscriber): Subscriber for obstacle safe dist state data.

        _received_collision (bool): Flag indicating if collision state data has been received.
        _received_ped_safe_dist (bool): Flag indicating if pedestrian safe dist state data has been 
            received.
        _received_obs_safe_dist (bool): Flag indicating if obstacle safe dist state data has been 
            received.

        _first_reset (bool): Flag indicating if it is the first reset.
    """

    # Retrieved information
    _collision: bool
    _ped_safe_dist: bool
    _obs_safe_dist: bool
    _image_depth: np.ndarray
    _image_color: np.ndarray

    # Subscriptions
    _collision_sub: rospy.Subscriber
    _ped_safe_dist_sub: rospy.Subscriber
    _obs_safe_dist_sub: rospy.Subscriber
    _image_depth_sub: rospy.Subscriber
    _image_color_sub: rospy.Subscriber

    # Received Flags
    _received_collision: bool
    _received_ped_safe_dist: bool
    _received_obs_safe_dist: bool
    _received_image_depth: bool
    _received_image_color: bool

    _first_reset: bool

    def __init__(self, ns: Namespace, observation_manager) -> None:
        """
        Initialize the UnityCollectorUnit.

        Args:
            ns (Namespace): Namespace for the collector unit.
            observation_manager: Observation manager holding this collector unit.
        """
        super().__init__(ns, observation_manager)
        
        self._enable_rgbd = rospy.get_param("rgbd/enabled", False)
        if self._enable_rgbd:
            self._image_bridge = CvBridge()

        self._collision = False
        self._ped_safe_dist = False
        self._obs_safe_dist = False

        self._collision_sub: rospy.Subscriber = None
        self._ped_safe_dist_sub: rospy.Subscriber = None
        self._obs_safe_dist_sub: rospy.Subscriber = None

        self._received_collision = False
        self._received_ped_safe_dist = False
        self._received_obs_safe_dist = False

        self._first_reset = True

    def init_subs(self):
        """
        Initialize the subscribers for robot state and sensor data.
        """
        self._collision_sub = rospy.Subscriber(
            self._ns(TOPICS.COLLISION),
            Collision,
            self._cb_collision,
            tcp_nodelay=True,
        )
        self._ped_safe_dist_sub = rospy.Subscriber(
            self._ns(TOPICS.PED_SAFE_DIST),
            Collision,
            self._cb_ped_safe_dist,
            tcp_nodelay=True,
        )
        self._obs_safe_dist_sub = rospy.Subscriber(
            self._ns(TOPICS.OBS_SAFE_DIST),
            Collision,
            self._cb_obs_safe_dist,
            tcp_nodelay=True,
        )
        if self._enable_rgbd:
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
            
    def _rgbd_wait_condition(self):
        return (
                    not self._enable_rgbd or 
                    (self._received_image_color and self._received_image_depth)
                )

    def wait(self):
        """
        Wait for the required data to be received.
        """
        if self._first_reset:
            self._first_reset = False
            return

        for _ in range(int(MAX_WAIT / SLEEP)):
            if (self._received_collision and
                self._received_ped_safe_dist and
                self._received_obs_safe_dist and
                self._rgbd_wait_condition()):
                return

            sleep(SLEEP)

        raise TimeoutError(
            f"Couldn't retrieve data for: {false_params(odom=self._received_collision, laser=self._received_ped_safe_dist, subgoal=self._received_obs_safe_dist)}"
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
                OBS_DICT_KEYS.COLLSION: self._collision,
                OBS_DICT_KEYS.PED_SAFE_DIST: self._ped_safe_dist,
                OBS_DICT_KEYS.OBS_SAFE_DIST: self._obs_safe_dist
            }
        )
        if self._enable_rgbd:
            obs_dict.update(
                {
                    OBS_DICT_KEYS.IMAGE_COLOR: self._image_color,
                    OBS_DICT_KEYS.IMAGE_DEPTH: self._image_depth
                }
            )

        return obs_dict

    def _cb_collision(self, collsion_msg: Collision):
        """
        Callback function for receiving collsion state data. Simply, safes the data.

        Args:
            collision_msg (Collision): Collision state data.
        """
        self._received_collision = True
        self._collision = collsion_msg.collision

    def _cb_ped_safe_dist(self, collsion_msg: Collision):
        """
        Callback function for receiving ped safe distance state data. Simply, safes the data.

        Args:
            collision_msg (Collision): Safe distance state data.
        """
        self._received_ped_safe_dist = True
        self._ped_safe_dist = collsion_msg.collision

    def _cb_obs_safe_dist(self, collsion_msg: Collision):
        """
        Callback function for receiving obs safe distance state data. Simply, safes the data.

        Args:
            collision_msg (Collision): Safe distance state data.
        """
        self._received_obs_safe_dist = True
        self._obs_safe_dist = collsion_msg.collision

    def _cb_image_color(self, image_msg: Image):
        """
        Callback function for receiving rgb data. Converts image msg to cv Mat to np ndarray.

        Args:
            image_msg (Image): Received rgb image message.
        """
        self._received_image_color = True
        cv_mat = self._image_bridge.imgmsg_to_cv2(image_msg)
        self._image_color = np.asarray(cv_mat)
        # let the channel dim be the first dim and remove a-channel
        self._image_color = (np.asarray(cv_mat)[:,:,0:3]).transpose((2, 0, 1))

    def _cb_image_depth(self, image_msg: Image):
        """Callback function for receiving depth data. Converts image msg to cv Mat to np ndarray.

        Args:
            image_msg (Image): Received depth image message.
        """
        self._received_image_depth = True
        cv_mat = self._image_bridge.imgmsg_to_cv2(image_msg)
        self._image_depth = np.asarray(cv_mat)
