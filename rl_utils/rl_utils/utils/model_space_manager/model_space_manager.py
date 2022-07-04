from typing import Any
from rl_utils.utils.model_space_manager.encoder_factory import BaseSpaceEncoderFactory
from .robot_specific_space_encoder import *
from .uniform_space_encoder import *

import rospy
import yaml

from gym import spaces
import numpy as np


"""
    Provides a uniform interface between model and environment.

    Offers encoders to scale observations to observation space.
    Offers the action and observation space sizes
"""

class ModelSpaceManager:
    def __init__(self):
        self._laser_num_beams = rospy.get_param("laser/num_beams")
        self._laser_max_range = rospy.get_param("laser/range")
        self._radius = rospy.get_param("robot_radius")
        self._is_holonomic = rospy.get_param("is_holonomic")
            
        encoder_name = rospy.get_param("space_encoder", "RobotSpecificEncoder")

        print(encoder_name)

        is_action_space_discrete = rospy.get_param("is_action_space_discrete", False)
        actions = rospy.get_param("actions/discrete") if is_action_space_discrete else rospy.get_param("actions/continuous")

        self._encoder = BaseSpaceEncoderFactory.instantiate(
            encoder_name, 
            self._laser_num_beams,
            self._laser_max_range,
            self._radius,
            self._is_holonomic,
            actions,
            is_action_space_discrete,
        )

    def get_observation_space(self):
        return self._encoder.get_observation_space()

    def get_action_space(self):
        return self._encoder.get_action_space()

    def encode_observation(self, observation):
        return self._encoder.encode_observation(observation)

    def decode_action(self, action):
        return self._encoder.decode_action(action)