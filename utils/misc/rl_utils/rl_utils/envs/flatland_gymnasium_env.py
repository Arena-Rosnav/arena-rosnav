#! /usr/bin/env python3
from typing import Tuple, Type

import gymnasium
import numpy as np
import rospy
from flatland_msgs.msg import StepWorld  # type: ignore
from geometry_msgs.msg import Twist
from rl_utils.utils.observation_collector import (
    DoneObservation,
    FullRangeLaserCollector,
    ObservationDict,
)
from rl_utils.utils.observation_collector.observation_manager import ObservationManager
from rl_utils.utils.rewards.reward_function import RewardFunction
from rosnav.model.base_agent import BaseAgent
from rosnav.rosnav_space_manager.rosnav_space_manager import RosnavSpaceManager
from rosnav.utils.observation_space import EncodedObservationDict
from std_srvs.srv import Empty
from rl_utils.topic import Namespace, Topic
from task_generator.task_generator_node import TaskGenerator
from task_generator.tasks import Task
from task_generator.utils import rosparam_get

from .utils import determine_termination


class FlatlandEnv(gymnasium.Env):
    """
    FlatlandEnv is an environment class that represents the Flatland environment for reinforcement learning.

    Args:
        ns (str): The namespace of the environment.
        agent_description (BaseAgent): The agent description.
        reward_fnc (str): The reward function.
        max_steps_per_episode (int): The maximum number of steps per episode. Default is 100.
        trigger_init (bool): Whether to trigger the initialization of the environment. Default is False.
        obs_unit_kwargs (dict): Additional keyword arguments for the observation unit. Default is None.
        reward_fnc_kwargs (dict): Additional keyword arguments for the reward function. Default is None.
        task_generator_kwargs (dict): Additional keyword arguments for the task generator. Default is None.
        *args: Additional positional arguments.
        **kwargs: Additional keyword arguments.

    Attributes:
        metadata (dict): The metadata of the environment.

    """

    metadata = {"render_modes": ["human"]}

    def __init__(
        self,
        ns: str,
        agent_description: BaseAgent,
        reward_fnc: str,
        max_steps_per_episode=100,
        init_by_call: bool = False,
        wait_for_obs: bool = False,
        obs_unit_kwargs=None,
        reward_fnc_kwargs=None,
        task_generator_kwargs=None,
        *args,
        **kwargs,
    ):
        """
        Initializes the FlatlandEnv class.

        Args:
            ns (str): The namespace for the environment.
            agent_description (BaseAgent): The agent description.
            reward_fnc (str): The reward function.
            max_steps_per_episode (int, optional): The maximum number of steps per episode. Defaults to 100.
            init_by_call (bool, optional): Whether to trigger initialization from outside. Defaults to False.
            obs_unit_kwargs (dict, optional): Additional keyword arguments for the observation unit. Defaults to None.
            reward_fnc_kwargs (dict, optional): Additional keyword arguments for the reward function. Defaults to None.
            task_generator_kwargs (dict, optional): Additional keyword arguments for the task generator. Defaults to None.
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.
        """
        super(FlatlandEnv, self).__init__()

        self.ns = Namespace(ns)
        self._agent_description = agent_description

        self._debug_mode = rospy.get_param("/debug_mode", False)
        self._is_train_mode = rospy.get_param("/train_mode", default=True)
        self._step_size = rospy.get_param("/step_size")

        if not self._debug_mode:
            rospy.init_node(f"env_{self.ns.simulation_ns}".replace("/", "_"))

        self._reward_fnc = reward_fnc
        self._reward_fnc_kwargs = reward_fnc_kwargs if reward_fnc_kwargs else {}
        self._obs_unit_kwargs = obs_unit_kwargs if obs_unit_kwargs else {}
        self._task_generator_kwargs = (
            task_generator_kwargs if task_generator_kwargs else {}
        )

        self._wait_for_obs = wait_for_obs

        self._steps_curr_episode = 0
        self._episode = 0
        self._max_steps_per_episode = max_steps_per_episode

        if not init_by_call:
            self.init()

    def init(self):
        """
        Initializes the environment.

        Returns:
            bool: True if the initialization is successful, False otherwise.
        """
        self.model_space_encoder = RosnavSpaceManager(
            ns=self.ns,
            space_encoder_class=self._agent_description.space_encoder_class,
            observation_spaces=self._agent_description.observation_spaces,
            observation_space_kwargs=self._agent_description.observation_space_kwargs,
        )

        if self._is_train_mode:
            self._setup_env_for_training(
                self._reward_fnc, **self._task_generator_kwargs
            )

        obs_structure = set(self.reward_calculator.required_observations).union(
            set(self.model_space_encoder.encoder.required_observations)
        )
        obs_structure.add(FullRangeLaserCollector)

        self.observation_collector = ObservationManager(
            ns=self.ns,
            obs_structur=list(obs_structure),
            obs_unit_kwargs=self._obs_unit_kwargs,
            wait_for_obs=self._wait_for_obs,
        )
        return True

    @property
    def action_space(self):
        """
        Returns the action space of the environment.

        Returns:
            action_space (object): The action space of the environment.
        """
        return self.model_space_encoder.get_action_space()

    @property
    def observation_space(self):
        """
        Returns the observation space of the environment.

        Returns:
            gym.Space: The observation space of the environment.
        """
        return self.model_space_encoder.get_observation_space()

    def _setup_env_for_training(self, reward_fnc: str, **kwargs):
        """
        Set up the environment for training.

        Args:
            reward_fnc (str): The name of the reward function.
            **kwargs: Additional keyword arguments.

        Returns:
            None
        """
        # instantiate task manager
        task_generator = TaskGenerator(self.ns.simulation_ns)
        self.task: Type[Task] = task_generator._get_predefined_task(**kwargs)

        # reward calculator
        self.reward_calculator = RewardFunction(
            rew_func_name=reward_fnc,
            holonomic=self.model_space_encoder._is_holonomic,
            robot_radius=self.task.robot_managers[0]._robot_radius,
            safe_dist=self.task.robot_managers[0].safe_distance,
            goal_radius=rosparam_get(float, "goal_radius", 0.3),
            distinguished_safe_dist=rosparam_get(
                bool, "rl_agent/distinguished_safe_dist", False
            ),
            ns=self.ns,
            max_steps=self._max_steps_per_episode,
            **self._reward_fnc_kwargs,
        )

        self.agent_action_pub = rospy.Publisher(
            str(self.ns("cmd_vel")), Twist, queue_size=1
        )

        # service clients
        self._service_name_step = str(self.ns.simulation_ns("step_world"))
        self._step_world_publisher = rospy.Publisher(
            self._service_name_step, StepWorld, queue_size=10
        )
        self._step_world_srv = rospy.ServiceProxy(
            self._service_name_step, Empty, persistent=True
        )

    def _pub_action(self, action: np.ndarray):
        """
        Publishes the given action to the agent's action topic.

        Args:
            action (np.ndarray): The action to be published. It should be a 1D numpy array of length 3,
                                 representing the linear x, linear y, and angular z components of the action.

        Raises:
            AssertionError: If the length of the action array is not 3.
        """
        assert len(action) == 3

        action_msg = Twist()
        action_msg.linear.x = action[0]
        action_msg.linear.y = action[1]
        action_msg.angular.z = action[2]

        self.agent_action_pub.publish(action_msg)

    def _decode_action(self, action: np.ndarray) -> np.ndarray:
        """
        Decodes the given action using the model space encoder.

        Args:
            action (np.ndarray): The action to be decoded.

        Returns:
            np.ndarray: The decoded action.
        """
        return self.model_space_encoder.decode_action(action)

    def _encode_observation(
        self, observation: ObservationDict, *args, **kwargs
    ) -> EncodedObservationDict:
        """
        Encodes the given observation using the model space encoder.

        Args:
            observation (ObservationDict): The observation to be encoded.

        Returns:
            The encoded observation.

        """
        return self.model_space_encoder.encode_observation(observation, **kwargs)

    def step(
        self, action: np.ndarray
    ) -> Tuple[EncodedObservationDict, float, bool, bool, dict]:
        """
        Take a step in the environment.

        Args:
            action (np.ndarray): The action to take.

        Returns:
            tuple: A tuple containing the encoded observation, reward, done flag, info dictionary, and False flag.

        """

        decoded_action = self._decode_action(action)
        self._pub_action(decoded_action)

        if self._is_train_mode:
            self._call_service_takeSimStep()

        obs_dict: ObservationDict = self.observation_collector.get_observations()

        # calculate reward
        reward, reward_info = self.reward_calculator.get_reward(obs_dict=obs_dict)

        self._steps_curr_episode += 1

        # info
        info, done = determine_termination(
            reward_info=reward_info,
            curr_steps=self._steps_curr_episode,
            max_steps=self._max_steps_per_episode,
        )

        return (
            self._encode_observation(obs_dict),
            reward,
            done,
            False,
            info,
        )

    def reset(self, seed=None, options=None) -> Tuple[EncodedObservationDict, dict]:
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

        self._before_task_reset()

        first_map = self._episode <= 1 if "sim_1" in self.ns else False

        self.task.reset(
            first_map=first_map,
            reset_after_new_map=self._steps_curr_episode == 0,
        )
        self.reward_calculator.reset()
        self._steps_curr_episode = 0

        self._after_task_reset()

        obs_dict = self.observation_collector.get_observations()
        obs_dict.update({DoneObservation.name: True})
        info_dict = {}
        return (
            self._encode_observation(obs_dict),
            info_dict,
        )

    def close(self):
        """
        Close the environment.

        """
        rospy.signal_shutdown("Closing environment...")

    def _before_task_reset(self):
        """
        Perform any necessary steps before resetting the task.

        """
        # make sure all simulation components are ready before first episode
        if self._episode <= 1:
            for _ in range(6):
                self.agent_action_pub.publish(Twist())
                self._call_service_takeSimStep()

    def _after_task_reset(self):
        """
        Perform any necessary steps after resetting the task.

        """
        if self._is_train_mode:
            # extra step for planning serivce to provide global plan
            for _ in range(4):
                self.agent_action_pub.publish(Twist())
                self._call_service_takeSimStep()

    def _call_service_takeSimStep(self, t: float = None, srv_call: bool = True):
        if srv_call:
            self._step_world_srv()
        request = StepWorld()
        request.required_time = self._step_size if t is None else t

        self._step_world_publisher.publish(request)
