from gym import spaces
import rospy
import numpy as np

from ..utils import stack_spaces
from .encoder_factory import BaseSpaceEncoderFactory
from .base_space_encoder import BaseSpaceEncoder


"""
    This encoder offers a robot specific observation and action space
    Different actions spaces for holonomic and non holonomic robots

    Observation space:   Laser Scan, Goal, Current Vel 
    Action space: X Vel, (Y Vel), Angular Vel

"""
@BaseSpaceEncoderFactory.register("RobotSpecificEncoder")
class RobotSpecificEncoder(BaseSpaceEncoder):
    def __init__(self, *args):
        super().__init__(*args)

    def decode_action(self, action):
        if self._is_action_space_discrete:
            return self._translate_disc_action(action)
        return self._extend_action_array(action)

    def _extend_action_array(self, action: np.ndarray) -> np.ndarray:
        if self._is_holonomic:
            assert (
                self._is_holonomic and len(action) == 3
            ), "Robot is holonomic but action with only two freedoms of movement provided"

            return action
        else:
            assert (
                not self._is_holonomic and len(action) == 2
            ), "Robot is non-holonomic but action with more than two freedoms of movement provided"
            return np.array([action[0], 0, action[1]])

    def _translate_disc_action(self, action):
        assert not self._is_holonomic, "Discrete action space currently not supported for holonomic robots"
        
        return np.array([
            self._actions[action]["linear"], 
            self._actions[action]["linear"]
        ])

    def encode_observation(self, observation):
        rho, theta = observation["goal_in_robot_frame"]
        scan = observation["laser_scan"]
        last_action = observation["last_action"]

        return np.hstack(
            [
                scan,
                np.array([rho, theta]),
                last_action,
            ]
        )

    def get_observation_space(self):
        return stack_spaces(
            spaces.Box(
                low=0,
                high=self._laser_max_range,
                shape=(self._laser_num_beams,),
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
        )

    def get_action_space(self):
        if self._is_action_space_discrete:
                # self._discrete_actions is a list, each element is a dict with the keys ["name", 'linear','angular']
                return spaces.Discrete(len(self._actions))

        linear_range = self._actions["linear_range"]
        angular_range = self._actions["angular_range"]

        if not self._is_holonomic:
            return spaces.Box(
                low=np.array([linear_range[0], angular_range[0]]),
                high=np.array([linear_range[1], angular_range[1]]),
                dtype=np.float32,
            )

        linear_range_x, linear_range_y = (
            linear_range["x"],
            linear_range["y"],
        )
        
        return spaces.Box(
            low=np.array(
                [
                    linear_range_x[0],
                    linear_range_y[0],
                    angular_range[0],
                ]
            ),
            high=np.array(
                [
                    linear_range_x[1],
                    linear_range_y[1],
                    angular_range[1],
                ]
            ),
            dtype=np.float32,
        )

