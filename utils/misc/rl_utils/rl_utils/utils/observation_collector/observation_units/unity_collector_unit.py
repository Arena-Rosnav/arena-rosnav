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
    A class for collecting Arena Unity specific collider data used only for reward functions.

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

    # Subscriptions
    _collision_sub: rospy.Subscriber
    _ped_safe_dist_sub: rospy.Subscriber
    _obs_safe_dist_sub: rospy.Subscriber

    # Received Flags
    _received_collision: bool
    _received_ped_safe_dist: bool
    _received_obs_safe_dist: bool

    _first_reset: bool

    def __init__(self, ns: Namespace, observation_manager, *args, **kwargs) -> None:
        """
        Initialize the UnityCollectorUnit.

        Args:
            ns (Namespace): Namespace for the collector unit.
            observation_manager: Observation manager holding this collector unit.
        """
        super().__init__(ns, observation_manager)
        
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

    def wait(self):
        """
        Wait for the required data to be received.
        """
        if self._first_reset:
            self._first_reset = False
            return

        for _ in range(int(MAX_WAIT / SLEEP)):
            if (self._received_collision
                and self._received_ped_safe_dist
                and self._received_obs_safe_dist):
                return

            sleep(SLEEP)

        raise TimeoutError(
            f"""Couldn't retrieve data for: {false_params(
                    collision=self._received_collision, 
                    ped_safe_dist=self._received_ped_safe_dist, 
                    obs_safe_dist=self._received_obs_safe_dist
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
                OBS_DICT_KEYS.COLLSION: self._collision,
                OBS_DICT_KEYS.PED_SAFE_DIST: self._ped_safe_dist,
                OBS_DICT_KEYS.OBS_SAFE_DIST: self._obs_safe_dist
            }
        )
        
        self._received_collision = False
        self._received_obs_safe_dist = False
        self._received_ped_safe_dist = False

        return obs_dict

    def _cb_collision(self, collsion_msg: Collision):
        """
        Callback function for receiving collsion state data. Simply, safes the data.

        Args:
            collision_msg (Collision): Collision state data.
        """
        self._collision = collsion_msg.collision
        self._received_collision = True

    def _cb_ped_safe_dist(self, collsion_msg: Collision):
        """
        Callback function for receiving ped safe distance state data. Simply, safes the data.

        Args:
            collision_msg (Collision): Safe distance state data.
        """
        self._ped_safe_dist = collsion_msg.collision
        self._received_ped_safe_dist = True

    def _cb_obs_safe_dist(self, collsion_msg: Collision):
        """
        Callback function for receiving obs safe distance state data. Simply, safes the data.

        Args:
            collision_msg (Collision): Safe distance state data.
        """
        self._obs_safe_dist = collsion_msg.collision
        self._received_obs_safe_dist = True
