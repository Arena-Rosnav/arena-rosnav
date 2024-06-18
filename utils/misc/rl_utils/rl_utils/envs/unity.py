from typing import Tuple

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from rl_utils.utils.arena_unity_utils.unity_timer import UnityTimer
from rl_utils.utils.observation_collector import ObservationDict
from rosgraph_msgs.msg import Clock
from rosnav.utils.observation_space import EncodedObservationDict

from .flatland_gymnasium_env import BaseAgent, FlatlandEnv, determine_termination


class UnityEnv(FlatlandEnv):
    """
    UnityEnv is a subclass of FlatlandEnv that represents an environment for Unity simulations.

    This class provides methods for setting up the environment for training, rendering, closing, and handling task resets.

    Attributes:
        _unity_timer (UnityTimer): A timer object for synchronizing with the Unity simulation clock.

    """

    def __init__(
        self,
        ns: str,
        agent_description: BaseAgent,
        reward_fnc: str,
        max_steps_per_episode=100,
        init_by_call: bool = False,
        wait_for_obs: bool = True,
        obs_unit_kwargs=None,
        reward_fnc_kwargs=None,
        task_generator_kwargs=None,
        *args,
        **kwargs,
    ):
        rospy.loginfo("[Unity Env ns:" + ns + "]: Starting intialization")
        super().__init__(
            ns,
            agent_description,
            reward_fnc,
            max_steps_per_episode,
            init_by_call,
            wait_for_obs,
            obs_unit_kwargs,
            reward_fnc_kwargs,
            task_generator_kwargs,
            *args,
            **kwargs,
        )
        rospy.loginfo("[Unity Env ns:" + ns + "]: Step size " + str(self._step_size))
        rospy.loginfo("[Unity Env ns:" + self.ns + "]: Intialization done")

    def _setup_env_for_training(self, reward_fnc: str, **kwargs):
        """
        Set up the environment for training.

        Args:
            reward_fnc (str): The name of the reward function to use.
            **kwargs: Additional keyword arguments.

        """
        # Unity specific
        clock_topic = self.ns.simulation_ns("clock")
        clock_msg = rospy.wait_for_message(clock_topic, Clock, timeout=30)
        self._unity_timer = UnityTimer(
            self._step_size,
            rospy.Time(clock_msg.clock.secs, clock_msg.clock.nsecs),
            clock_topic,
        )
        super()._setup_env_for_training(reward_fnc, **kwargs)

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
            self._unity_timer.wait_for_next_update()

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

    def render(self):
        """
        Render the environment.

        This method is currently empty.

        """
        pass

    def close(self):
        """
        Close the environment.

        This method logs a message indicating that the environment is being closed.

        """
        rospy.loginfo("[Unity Env ns:" + self.ns + "]: Closing environment.")
        rospy.signal_shutdown("Closing Unity environment.")

    def _before_task_reset(self):
        """
        Perform actions before resetting the task.

        This method publishes a Twist message with zero values to the agent_action_pub topic.

        """
        self.agent_action_pub.publish(Twist())

    def _after_task_reset(self):
        """
        Perform actions after resetting the task.

        If the environment is in train mode, this method publishes a Twist message with zero values to the agent_action_pub topic
        and waits for the next update using the _unity_timer.

        """
        if self._is_train_mode:
            self.agent_action_pub.publish(Twist())
            self._unity_timer.wait_for_next_update()
