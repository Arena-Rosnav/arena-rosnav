#! /usr/bin/env python3
import math
import os
import time
from typing import Tuple

import gymnasium
import numpy as np
from rosnav.model.base_agent import BaseAgent
import rospy
from flatland_msgs.msg import StepWorld
from geometry_msgs.msg import Twist
from rosnav.rosnav_space_manager.rosnav_space_manager import RosnavSpaceManager
from stable_baselines3.common.env_checker import check_env
from std_srvs.srv import Empty
from task_generator.shared import Namespace
from task_generator.task_generator_node import TaskGenerator
from task_generator.tasks.base_task import BaseTask
from task_generator.utils import rosparam_get

# from ..utils.old_observation_collector import ObservationCollector
from rl_utils.utils.observation_collector.observation_manager import ObservationManager
from rl_utils.utils.rewards.reward_function import RewardFunction
from rl_utils.utils.observation_collector.constants import OBS_DICT_KEYS


def delay_node_init(ns):
    try:
        # given every environment enough time to initialize, if we dont put sleep,
        # the training script may crash.
        import re

        ns_int = int(re.search(r"\d+", ns)[0])
        time.sleep((ns_int + 1) * 2)
    except Exception:
        rospy.logwarn(
            "Can't not determinate the number of the environment, training script may crash!"
        )
        time.sleep(2)


class FlatlandEnv(gymnasium.Env):
    """
    FlatlandEnv is an environment class that represents a Flatland environment for reinforcement learning.

    Args:
        ns (str): The namespace of the environment.
        reward_fnc (str): The name of the reward function.
        max_steps_per_episode (int): The maximum number of steps per episode.
        verbose (bool): Whether to print verbose information.
        log_last_n_eps (int): The number of episodes to log statistics for.

    Attributes:
        metadata (dict): Metadata for the environment.
        ns (Namespace): The namespace of the environment.
        _is_train_mode (bool): Whether the environment is in training mode.
        model_space_encoder (RosnavSpaceManager): The space encoder for the model.
        observation_collector (ObservationManager): The observation collector.
        task (BaseTask): The task manager.
        reward_calculator (RewardFunction): The reward calculator.
        agent_action_pub (Publisher): The publisher for agent actions.
        _service_name_step (str): The name of the step world service.
        _step_world_srv (ServiceProxy): The service proxy for the step world service.
        _verbose (bool): Whether to print verbose information.
        _log_last_n_eps (int): The number of episodes to log statistics for.
        _steps_curr_episode (int): The current number of steps in the episode.
        _episode (int): The current episode number.
        _max_steps_per_episode (int): The maximum number of steps per episode.
        _last_action (np.ndarray): The last action taken by the agent.
        last_mean_reward (float): The mean reward of the last logged episodes.
        mean_reward (list): The cumulative reward and count of episodes.
        step_count_hist (list): The history of step counts for the last logged episodes.
        step_time (list): The cumulative step time and count of steps.
        _done_reasons (dict): The reasons for episode termination.
        _done_hist (list): The history of episode terminations.

    Properties:
        action_space: The action space of the environment.
        observation_space: The observation space of the environment.

    Methods:
        _setup_env_for_training: Set up the environment for training.
        _pub_action: Publish an action to the agent.
        decode_action: Decode an action from the model space.
        encode_observation: Encode an observation into the model space.
        step: Take a step in the environment.
        call_service_takeSimStep: Call the step world service.
        reset: Reset the environment.
        close: Close the environment.
        update_statistics: Update the statistics of the environment.
        print_statistics: Print the statistics of the environment.
        determine_termination: Determine if the episode is terminated.

    """

    metadata = {"render_modes": ["human"]}

    def __init__(
        self,
        ns: str,
        agent_description: BaseAgent,
        reward_fnc: str,
        max_steps_per_episode=100,
        verbose: bool = True,
        log_last_n_eps: int = 20,
        *args,
        **kwargs,
    ):
        super(FlatlandEnv, self).__init__()

        self.ns = Namespace(ns)
        self._agent_description = agent_description

        delay_node_init(ns=self.ns.simulation_ns)

        if not rospy.get_param("/debug_mode", True):
            rospy.init_node("env_" + self.ns, anonymous=True)

        self._is_train_mode = rospy.get_param("/train_mode")
        self.model_space_encoder = RosnavSpaceManager(
            space_encoder_class=self._agent_description.space_encoder_class,
            observation_spaces=self._agent_description.observation_spaces,
            observation_space_kwargs=self._agent_description.observation_space_kwargs,
        )

        if self._is_train_mode:
            self._setup_env_for_training(reward_fnc, **kwargs)

        # observation collector
        self.observation_collector = ObservationManager(self.ns)

        self._verbose = verbose
        self._log_last_n_eps = log_last_n_eps

        self._steps_curr_episode = 0
        self._episode = 0
        self._max_steps_per_episode = max_steps_per_episode
        self._last_action = np.array([0, 0, 0])  # linear x, linear y, angular z

        # for extended eval
        self.last_mean_reward = 0
        self.mean_reward = [0, 0]
        self.step_count_hist = [0] * self._log_last_n_eps
        self.step_time = [0, 0]

        self._done_reasons = {
            "0": "Timeout",
            "1": "Crash",
            "2": "Success",
        }
        self._done_hist = 3 * [0]

    @property
    def action_space(self):
        return self.model_space_encoder.get_action_space()

    @property
    def observation_space(self):
        return self.model_space_encoder.get_observation_space()

    def _setup_env_for_training(self, reward_fnc: str, **kwargs):
        # instantiate task manager
        task_generator = TaskGenerator(self.ns.simulation_ns)
        self.task: BaseTask = task_generator._get_predefined_task(**kwargs)

        # reward calculator
        self.reward_calculator = RewardFunction(
            rew_func_name=reward_fnc,
            holonomic=self.model_space_encoder._is_holonomic,
            robot_radius=self.task.robot_managers[0]._robot_radius,
            safe_dist=self.task.robot_managers[0].safe_distance,
            goal_radius=rosparam_get(float, "goal_radius", 0.3),
        )

        self.agent_action_pub = rospy.Publisher(self.ns("cmd_vel"), Twist, queue_size=1)

        # service clients
        self._service_name_step = self.ns.simulation_ns("step_world")
        self._step_world_srv = rospy.ServiceProxy(
            self._service_name_step, Empty, persistent=True
        )

    def _pub_action(self, action: np.ndarray) -> Twist:
        assert len(action) == 3

        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.linear.y = action[1]
        action_msg.angular.z = action[2]

        self.agent_action_pub.publish(action_msg)

    def decode_action(self, action: np.ndarray) -> np.ndarray:
        return self.model_space_encoder.decode_action(action)

    def encode_observation(self, observation, structure=None):
        return self.model_space_encoder.encode_observation(observation, structure)

    def step(self, action: np.ndarray):
        """
        Take a step in the environment.

        Args:
            action (np.ndarray): The action to take.

        Returns:
            tuple: A tuple containing the encoded observation, reward, done flag, info dictionary, and False flag.

        """

        start_time = time.time()

        decoded_action = self.decode_action(action)
        self._pub_action(decoded_action)

        if self._is_train_mode:
            self.call_service_takeSimStep()

        obs_dict = self.observation_collector.get_observations(
            last_action=self._last_action
        )
        self._last_action = decoded_action

        # calculate reward
        reward, reward_info = self.reward_calculator.get_reward(
            action=decoded_action,
            **obs_dict,
        )

        self.update_statistics(reward=reward)

        # info
        info, done = FlatlandEnv.determine_termination(
            reward_info=reward_info,
            curr_steps=self._steps_curr_episode,
            max_steps=self._max_steps_per_episode,
        )

        if done and self._verbose:
            self.step_count_hist[
                self._episode % self._log_last_n_eps
            ] = self._steps_curr_episode
            self._done_hist[int(info["done_reason"])] += 1
            if sum(self._done_hist) >= self._log_last_n_eps:
                self.print_statistics()

        self.step_time[0] += time.time() - start_time

        return (
            self.encode_observation(obs_dict),
            reward,
            done,
            False,
            info,
        )

    def call_service_takeSimStep(self, t=None):
        # request = StepWorld()
        # request.required_time = 0 if t == None else t

        self._step_world_srv()

        # self._step_world_publisher.publish(request)

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
        # set task
        # regenerate start position end goal position of the robot and change the obstacles accordingly
        self._episode += 1
        self.agent_action_pub.publish(Twist())

        first_map = self._episode <= 1 if "sim_1" in self.ns else False
        self.task.reset(
            callback=lambda: False,
            first_map=first_map,
            reset_after_new_map=self._steps_curr_episode == 0,
        )
        self.reward_calculator.reset()
        self._steps_curr_episode = 0
        self._last_action = np.array([0, 0, 0])

        if self._is_train_mode:
            for _ in range(7):
                self.call_service_takeSimStep()

        obs_dict = self.observation_collector.get_observations()
        info_dict = {}
        return (
            self.model_space_encoder.encode_observation(
                obs_dict,
            ),
            info_dict,
        )

    def close(self):
        """
        Close the environment.

        """

        pass

    def update_statistics(self, **kwargs) -> None:
        """
        Update the statistics of the environment.

        Args:
            **kwargs: Additional keyword arguments.

        """

        self.step_time[1] += 1
        self.mean_reward[1] += 1
        self.mean_reward[0] += kwargs["reward"]
        self._steps_curr_episode += 1

    def print_statistics(self):
        """
        Print the statistics of the environment.

        """

        mean_reward = self.mean_reward[0] / self._log_last_n_eps
        diff = round(mean_reward - self.last_mean_reward, 5)

        print(
            f"[{self.ns}] Last {self._log_last_n_eps} Episodes:\t"
            f"{self._done_reasons[str(0)]}: {self._done_hist[0]}\t"
            f"{self._done_reasons[str(1)]}: {self._done_hist[1]}\t"
            f"{self._done_reasons[str(2)]}: {self._done_hist[2]}\t"
            f"Mean step time: {round(self.step_time[0] / self.step_time[1] * 100, 2)}\t"
            f"Mean cum. reward: {round(mean_reward, 5)} ({'+' if diff >= 0 else ''}{diff})\t"
            f"Mean steps: {sum(self.step_count_hist) / self._log_last_n_eps}\t"
        )
        self._done_hist = [0] * 3
        self.step_time = [0, 0]
        self.last_mean_reward = mean_reward
        self.mean_reward = [0, 0]
        self.step_count_hist = [0] * self._log_last_n_eps

    @staticmethod
    def determine_termination(
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
            tuple: A tuple containing the info dictionary and a boolean flag indicating if the episode should terminate.

        """

        if info is None:
            info = {}

        terminated = reward_info["is_done"]

        if terminated:
            info["done_reason"] = reward_info["done_reason"]
            info["is_success"] = reward_info["is_success"]

        if curr_steps >= max_steps:
            terminated = True
            info["done_reason"] = 0
            info["is_success"] = 0

        return info, terminated
