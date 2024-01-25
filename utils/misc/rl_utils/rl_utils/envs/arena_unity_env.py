#! /usr/bin/env python3
import re
import time
import random

from typing import Tuple

import gymnasium
import numpy as np
import rospy
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from rl_utils.utils.observation_collector.constants import DONE_REASONS
from rl_utils.utils.observation_collector.observation_manager import ObservationManager
from rl_utils.utils.rewards.reward_function import RewardFunction
from rl_utils.utils.arena_unity_utils.unity_timer import UnityTimer
from rosnav.model.base_agent import BaseAgent
from rosnav.rosnav_space_manager.rosnav_space_manager import RosnavSpaceManager
from std_srvs.srv import Empty
from task_generator.shared import Namespace
from task_generator.task_generator_node import TaskGenerator
from task_generator.utils import rosparam_get


def get_ns_idx(ns: str):
    try:
        return int(re.search(r"\d+", ns)[0])
    except Exception:
        return random.uniform(0, 3)
        # return 0.5


class ArenaUnityEnv(gymnasium.Env):
    """That's an environment for Arena Unity
    """
    
    metadata = {"render_modes": ["human"]}

    def __init__(
        self,
        ns: str,
        agent_description: BaseAgent,
        reward_fnc: str,
        max_steps_per_episode=100,
        trigger_init: bool = False,
        *args,
        **kwargs,
    ):
        """
        Just sets all the given parameters as properties
        """
        super(ArenaUnityEnv, self).__init__()

        self.ns = Namespace(ns)
        self._agent_description = agent_description

        self._debug_mode = rospy.get_param("/debug_mode", False)

        if not self._debug_mode:
            rospy.init_node(f"env_{self.ns.simulation_ns}".replace("/", "_"))

        self._is_train_mode = rospy.get_param_cached("/train_mode", default=True)
        self._step_size = rospy.get_param_cached("/step_size")

        self._reward_fnc = reward_fnc
        self._kwargs = kwargs

        self._steps_curr_episode = 0
        self._episode = 0
        self._max_steps_per_episode = max_steps_per_episode
        self._last_action = np.array([0, 0, 0])  # linear x, linear y, angular z

        if not trigger_init:
            self.init()

    def init(self):
        """
        Initializes the environment.
        Intializes space encoders.

        Returns:
            bool: True if the initialization is successful, False otherwise.
        """
        self.model_space_encoder = RosnavSpaceManager(
            space_encoder_class=self._agent_description.space_encoder_class,
            observation_spaces=self._agent_description.observation_spaces,
            observation_space_kwargs=self._agent_description.observation_space_kwargs,
        )

        if self._is_train_mode:
            self._setup_env_for_training(self._reward_fnc, **self._kwargs)

        # observation collector
        self.observation_collector = ObservationManager(self.ns)
        return True

    @property
    def action_space(self):
        return self.model_space_encoder.get_action_space()

    @property
    def observation_space(self):
        return self.model_space_encoder.get_observation_space()

    def _setup_env_for_training(self, reward_fnc: str, **kwargs):
        # instantiate task manager
        task_generator = TaskGenerator(self.ns.simulation_ns)
        self.task = task_generator._get_predefined_task(**kwargs)

        # reward calculator
        self.reward_calculator = RewardFunction(
            rew_func_name=reward_fnc,
            holonomic=self.model_space_encoder._is_holonomic,
            robot_radius=self.task.robot_managers[0]._robot_radius,
            safe_dist=self.task.robot_managers[0].safe_distance,
            goal_radius=rosparam_get(float, "goal_radius", 0.3),
        )

        self.agent_action_pub = rospy.Publisher(self.ns("cmd_vel"), Twist, queue_size=1)

        # Unity specific 
        clock_topic = self.ns.simulation_ns("clock")       
        self._unity_timer = UnityTimer(
            self._step_size,
            rospy.wait_for_message(clock_topic, Clock, timeout=10),
            clock_topic
        )

    def _pub_action(self, action: np.ndarray) -> Twist:
        assert len(action) == 3

        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.linear.y = action[1]
        action_msg.angular.z = action[2]

        self.agent_action_pub.publish(action_msg)

    def _decode_action(self, action: np.ndarray) -> np.ndarray:
        return self.model_space_encoder.decode_action(action)

    def _encode_observation(self, observation, *args, **kwargs):
        return self.model_space_encoder.encode_observation(observation, **kwargs)

    def step(self, action: np.ndarray):
        """
        Take a step in the environment.

        Args:
            action (np.ndarray): The action to take.

        Returns:
            tuple: A tuple containing the encoded observation, reward, done flag, info dictionary, and False flag.

        """
        if self._is_train_mode:
            self._unity_timer.wait_for_next_update()

        decoded_action = self._decode_action(action)
        self._pub_action(decoded_action)

        obs_dict = self.observation_collector.get_observations(
            last_action=self._last_action
        )
        self._last_action = decoded_action

        # calculate reward
        reward, reward_info = self.reward_calculator.get_reward(
            action=decoded_action,
            **obs_dict,
        )

        self._steps_curr_episode += 1

        # info
        info, done = self._determine_termination(
            reward_info=reward_info,
            curr_steps=self._steps_curr_episode,
            max_steps=self._max_steps_per_episode,
        )

        return (
            self._encode_observation(obs_dict, is_done=done),
            reward,
            done,
            False,
            info,
        )

    def unity_clock_cb(self, time_msg: Clock):
        pass

    def reset(self, seed=None, options=None):
        """
        Reset the environment.

        Args:
            seed: The random seed for the environment.
            options: Additional options for resetting the environment.

        Returns:
            tuple: A tuple containing the encoded observation and an empty info dictionary.

        """

        super().reset(seed=seed)
        self._episode += 1
        self.agent_action_pub.publish(Twist())

        first_map = self._episode <= 1 if "sim_1" in self.ns else False
        self.task.reset(
            first_map=first_map,
            reset_after_new_map=self._steps_curr_episode == 0,
        )
        self.reward_calculator.reset()
        self._steps_curr_episode = 0
        self._last_action = np.array([0, 0, 0])

        if self._is_train_mode:
            self.agent_action_pub.publish(Twist())
            self._unity_timer.wait_for_next_update()

        obs_dict = self.observation_collector.get_observations()
        info_dict = {}
        return (
            self._encode_observation(obs_dict),
            info_dict,
        )

    def close(self):
        """
        Close the environment.

        """
        pass

    def _determine_termination(
        self,
        reward_info: dict,
        curr_steps: int,
        max_steps: int,
        info: dict = None,
    ) -> Tuple[dict, bool]:
        """
        Determine if the episode should terminate.

        Args:
            reward_info (dict): The reward information.
            curr_steps (int): The current number of steps in the episode.
            max_steps (int): The maximum number of steps per episode.
            info (dict): Additional information.

        Returns:
            tuple: A tuple containing the info dictionary and a boolean flag indicating if 
            the episode should terminate.

        """

        if info is None:
            info = {}

        terminated = reward_info["is_done"]

        if terminated:
            info["done_reason"] = reward_info["done_reason"]
            info["is_success"] = reward_info["is_success"]
            info["episode_length"] = self._steps_curr_episode

        if curr_steps >= max_steps:
            terminated = True
            info["done_reason"] = DONE_REASONS.STEP_LIMIT.name
            info["is_success"] = 0
            info["episode_length"] = self._steps_curr_episode

        return info, terminated
    
    def render(self):
        pass
