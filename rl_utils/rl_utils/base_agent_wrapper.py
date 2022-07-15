from abc import ABC, abstractmethod
from typing import Tuple

import json
import numpy as np
import os
import rospy
import rospkg
import yaml

from gym import spaces

from geometry_msgs.msg import Twist

from rl_utils.utils.constants import Constants
from rosnav.model_space_manager.encoder_factory import BaseSpaceEncoderFactory
from rosnav.model_space_manager.model_space_manager import ModelSpaceManager

from .utils.utils import get_default_hyperparams_path
from .utils.observation_collector import ObservationCollector
from .utils.reward import RewardCalculator


class BaseDRLAgent(ABC):
    def __init__(
        self,
        ns: str = None,
        robot_name: str = None,
        hyperparameter_path: str = get_default_hyperparams_path(),
        *args,
        **kwargs,
    ) -> None:
        """[summary]

        Args:
            ns (str, optional):
                Agent name (directory has to be of the same name). Defaults to None.
            robot_name (str, optional):
                Robot specific ROS namespace extension. Defaults to None.
            hyperparameter_path (str, optional):
                Path to json file containing defined hyperparameters.
                Defaults to DEFAULT_HYPERPARAMETER.
        """
        self._is_train_mode = rospy.get_param("/train_mode")

        self._ns = "" if ns is None or ns == "" else ns + "/"
        self._ns_robot = (
            self._ns if robot_name is None else self._ns + robot_name + "/"
        )
        self._robot_sim_ns = robot_name

        self._num_laser_beams = rospy.get_param("/laser/num_beams")
        self._laser_range = rospy.get_param("/laser/range")
        self._robot_radius = rospy.get_param("robot_radius") * 1.05
        self._holonomic = rospy.get_param("is_holonomic")
        self._discrete_actions = rospy.get_param("actions/discrete")
        self._cont_actions = rospy.get_param("actions/continuous")

        self.model_space_encoder = ModelSpaceManager()

        self.load_hyperparameters(path=hyperparameter_path)
        self.setup_reward_calculator()

        self.observation_collector = ObservationCollector(
            self._ns_robot, self._num_laser_beams, external_time_sync=False
        )

        # for time controlling in train mode
        self._action_frequency = 1 / rospy.get_param("/robot_action_rate")

        if self._is_train_mode:
            # w/o action publisher node
            self._action_pub = rospy.Publisher(
                f"{self._ns_robot}cmd_vel", Twist, queue_size=1
            )
        else:
            # w/ action publisher node
            # (controls action rate being published on '../cmd_vel')
            self._action_pub = rospy.Publisher(
                f"{self._ns_robot}cmd_vel_pub", Twist, queue_size=1
            )

    @abstractmethod
    def setup_agent(self) -> None:
        """Sets up the new agent / loads a pretrained one.

        Raises:
            NotImplementedError: Abstract method.
        """
        raise NotImplementedError

    def load_hyperparameters(self, path: str) -> None:
        """Loads the hyperparameters from a json file.

        Args:
            path (str): Path to the json file.
        """
        assert os.path.isfile(
            path
        ), f"Hyperparameters file cannot be found at {path}!"

        with open(path, "r") as file:
            hyperparams = json.load(file)

        self._agent_params = hyperparams
        self._get_robot_name_from_params()

        import rosnav.model.custom_policy
        import rosnav.model.custom_sb3_policy

    def _get_robot_name_from_params(self):
        """Retrives the agent-specific robot name from the dictionary loaded\
            from respective 'hyperparameter.json'.    
        """
        assert self._agent_params and self._agent_params["robot"]
        self.robot_config_name = self._agent_params["robot"]

    def setup_reward_calculator(self) -> None:
        """Sets up the reward calculator."""
        assert self._agent_params and "reward_fnc" in self._agent_params
        self.reward_calculator = RewardCalculator(
            holonomic=self._holonomic,
            robot_radius=self._robot_radius,
            safe_dist=1.6 * self._robot_radius,
            goal_radius=Constants.GOAL_RADIUS,
            rule=self._agent_params["reward_fnc"],
            extended_eval=False,
        )

    @property
    def action_space(self) -> spaces.Box:
        """Returns the DRL agent's action space.

        Returns:
            spaces.Box: Agent's action space
        """
        return self.model_space_encoder.get_action_space()

    @property
    def observation_space(self) -> spaces.Box:
        """Returns the DRL agent's observation space.

        Returns:
            spaces.Box: Agent's observation space
        """
        return self.model_space_encoder.get_observation_space()

    def get_observations(self) -> Tuple[np.ndarray, dict]:
        """Retrieves the latest synchronized observation.

        Returns:
            Tuple[np.ndarray, dict]: 
                Tuple, where first entry depicts the observation data concatenated \
                into one array. Second entry represents the observation dictionary.
        """
        obs_dict = self.observation_collector.get_observations()

        merged_obs = self.model_space_encoder.encode_observation(obs_dict)

        if self._agent_params["normalize"]:
            merged_obs = self.normalize_observations(merged_obs)
        return merged_obs, obs_dict

    def normalize_observations(self, merged_obs: np.ndarray) -> np.ndarray:
        """Normalizes the observations with the loaded VecNormalize object.

        Note:
            VecNormalize object from Stable-Baselines3 is agent specific\
            and integral part in order to map right actions.\

        Args:
            merged_obs (np.ndarray):
                observation data concatenated into one array.

        Returns:
            np.ndarray: Normalized observations array.
        """
        assert self._agent_params["normalize"] and hasattr(
            self, "_obs_norm_func"
        )
        return self._obs_norm_func(merged_obs)

    def get_action(self, obs: np.ndarray) -> np.ndarray:
        """Infers an action based on the given observation.

        Args:
            obs (np.ndarray): Merged observation array.

        Returns:
            np.ndarray:
                Action in [linear velocity, angular velocity]
        """
        assert self._agent, "Agent model not initialized!"
        action = self._agent.predict(obs, deterministic=True)[0]

        return self.model_space_encoder.decode_action(action)

    def get_reward(self, action: np.ndarray, obs_dict: dict) -> float:
        """Calculates the reward based on the parsed observation

        Args:
            action (np.ndarray):
                Velocity commands of the agent\
                in [linear velocity, angular velocity].
            obs_dict (dict):
                Observation dictionary where each key makes up a different \
                kind of information about the environment.
        Returns:
            float: Reward amount
        """
        return self.reward_calculator.get_reward(action=action, **obs_dict)

    def publish_action(self, action: np.ndarray) -> None:
        """Publishes an action on 'self._action_pub' (ROS topic).

        Args:
            action (np.ndarray):
                Action in [linear velocity, angular velocity]
        """
        action_msg = self._get_action_msg(action)
        self._action_pub.publish(action_msg)

    def _get_disc_action(self, action: int) -> np.ndarray:
        """Returns defined velocity commands for parsed action index.\
            (Discrete action space)

        Args:
            action (int): Index of the desired action.

        Returns:
            np.ndarray: Velocity commands corresponding to the index.
        """
        return np.array(
            [
                self._discrete_actions[action]["linear"],
                self._discrete_actions[action]["angular"],
            ]
        )

    def _get_action_msg(self, action):
        action_msg = Twist()
        
        action_msg.linear.x = action[0]
        action_msg.linear.y = action[1]
        action_msg.angular.z = action[2]

        return action_msg

    def _get_hol_action_msg(self, action: np.ndarray):
        assert (
            len(action) == 3
        ), "Holonomic robots require action arrays to have 3 entries."
        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.linear.y = action[1]
        action_msg.angular.z = action[2]
        return action_msg

    def _get_nonhol_action_msg(self, action: np.ndarray):
        assert (
            len(action) == 2
        ), "Non-holonomic robots require action arrays to have 2 entries."
        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.angular.z = action[1]
        return action_msg
