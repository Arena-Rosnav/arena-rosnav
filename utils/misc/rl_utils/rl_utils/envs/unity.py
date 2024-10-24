from typing import Optional, Tuple, Type, Union

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from rl_utils.state_container import SimulationStateContainer
from rl_utils.topic import Namespace
from rl_utils.utils.arena_unity_utils.unity_timer import UnityTimer
from rl_utils.utils.type_alias.observation import ObservationDict
from rosgraph_msgs.msg import Clock
from rosnav_rl.reward.reward_function import RewardFunction
from rosnav_rl.spaces import EncodedObservationDict, RosnavSpaceManager

from .flatland_gymnasium_env import FlatlandEnv
from .utils import determine_termination


class UnityEnv(FlatlandEnv):
    """
    UnityEnv is a subclass of FlatlandEnv that represents an environment for Unity simulations.

    This class provides methods for setting up the environment for training, rendering, closing, and handling task resets.

    Attributes:
        _unity_timer (UnityTimer): A timer object for synchronizing with the Unity simulation clock.

    """

    def __init__(
        self,
        ns: Union[str, Namespace],
        space_manager: RosnavSpaceManager,
        reward_function: Optional[RewardFunction] = None,
        simulation_state_container: Optional[SimulationStateContainer] = None,
        max_steps_per_episode=100,
        init_by_call: bool = False,
        wait_for_obs: bool = False,
        obs_unit_kwargs=None,
        task_generator_kwargs=None,
        *args,
        **kwargs,
    ):
        rospy.loginfo("[Unity Env:" + ns + "]: Starting intialization")
        super().__init__(
            ns,
            space_manager,
            reward_function,
            simulation_state_container,
            max_steps_per_episode,
            init_by_call,
            wait_for_obs,
            obs_unit_kwargs,
            task_generator_kwargs,
            *args,
            **kwargs,
        )
        rospy.loginfo("[Unity Env:" + ns + "]: Step size " + str(self._step_size))
        rospy.loginfo("[Unity Env:" + self.ns + "]: Intialization done")

    def _setup_env_for_training(self, **kwargs):
        # Unity specific
        clock_topic = self.ns.simulation_ns("clock")
        clock_msg = rospy.wait_for_message(clock_topic, Clock, timeout=30)
        self._unity_timer = UnityTimer(
            self._step_size,
            rospy.Time(clock_msg.clock.secs, clock_msg.clock.nsecs),
            clock_topic,
        )
        super()._setup_env_for_training()

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
        reward, reward_info = self._reward_function.get_reward(obs_dict=obs_dict)

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
