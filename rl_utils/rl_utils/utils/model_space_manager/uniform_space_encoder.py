from random import sample
from gym import spaces
import rospy
import math
import numpy as np
from scipy import interpolate

from ..utils import stack_spaces
from .encoder_factory import BaseSpaceEncoderFactory
from .base_space_encoder import BaseSpaceEncoder

"""
    This encoder represents a uniform observation and action space on the model
    that can be used by every robot without any adaptations.

    All laser scans should start at angle 0
    Lasers that do not exist should be filled with max laser range

"""

LASER_SCANS = 1200


@BaseSpaceEncoderFactory.register("UniformSpaceEncoder")
class UniformSpaceEncoder(BaseSpaceEncoder):
    def __init__(self, *args):
        super().__init__(*args)

        self._laser_angle_min = rospy.get_param("laser/angle/min")
        self._laser_angle_max = rospy.get_param("laser/angle/max")
        self._laser_angle_increment = rospy.get_param("laser/angle/increment")

        self._missing_lasers = int((2 * math.pi - abs(self._laser_angle_min) - abs(self._laser_angle_max)) / self._laser_num_beams)

        self._lasers_right = int(abs(self._laser_angle_min) / self._laser_angle_increment)

        self.max_velocities = np.array(self._get_max_velocity())

    def decode_action(self, action):
        new_action = []
        
        for i in range(len(action)):
            a = action[i]
            adder = 1 if a >= 0 else 0
            new_action.append(self.max_velocities[i * 2 + adder] * a)

        return new_action

    def encode_observation(self, observation):
        rho, theta = observation["goal_in_robot_frame"]
        scan = observation["laser_scan"]
        last_action = observation["last_action"]

        right_lasers = scan[:self._lasers_right]
        left_lasers = scan[self._lasers_right:]

        lasers = np.hstack([
            np.array(left_lasers), 
            # np.full(self._missing_lasers, self._laser_max_range), 
            np.array(right_lasers)]
        )

        ## Scale lasers to LASER_SCAN
        f = interpolate.interp1d(np.arange(0, len(lasers)), lasers)
        sampled_lasers = f(np.linspace(0, len(lasers) - 1, LASER_SCANS))

        return np.hstack(
            [
                sampled_lasers, np.array([rho, theta]), last_action, self.max_velocities
            ]
        )


    def get_observation_space(self):
        return stack_spaces(
            spaces.Box(
                low=0,
                high=self._laser_max_range,
                shape=(LASER_SCANS,),
                dtype=np.float32,
            ),
            spaces.Box(low=0, high=15, shape=(1,), dtype=np.float32),
            spaces.Box(
                low=-np.pi, high=np.pi, shape=(1,), dtype=np.float32
            ),
            spaces.Box(
                low=-2.0,
                high=2.0,
                shape=(2,),
                dtype=np.float32,  # linear vel
            ),
            spaces.Box(
                low=-4.0,
                high=4.0,
                shape=(1,),
                dtype=np.float32,  # angular vel
            ),
            spaces.Box(  ## All max velocities
                low=-10,
                high=10,
                shape=(6,), # [-x, x, -y, y, -angle, angle]
                dtype=np.float32
            )
        )

    def get_action_space(self):
        return spaces.Box(
            low=np.array([-1, -1, -1]),
            high=np.array([1, 1, 1]),
            dtype=np.float32,
        )

    def _get_max_velocity(self):
        assert not self._is_action_space_discrete, "Discrete action space is not supported for uniform interface"

        linear_range = self._actions["linear_range"]
        angular_range = self._actions["angular_range"]

        angle = [angular_range[0], angular_range[1]]

        if not self._is_holonomic:
            x = [linear_range[0], linear_range[1]]
            y = [0, 0]
        else:
            linear_range_x, linear_range_y = (
                linear_range["x"],
                linear_range["y"],
            )

            x = [linear_range_x[0], linear_range_x[1]]
            y = [linear_range_y[0], linear_range_y[1]]

        return [*x, *y, *angle]