import rospy
from geometry_msgs.msg import Twist
from rl_utils.utils.arena_unity_utils.unity_timer import UnityTimer
from rosgraph_msgs.msg import Clock

from .flatland_gymnasium_env import FlatlandEnv


class UnityEnv(FlatlandEnv):
    """
    UnityEnv is a subclass of FlatlandEnv that represents an environment for Unity simulations.

    This class provides methods for setting up the environment for training, rendering, closing, and handling task resets.

    Attributes:
        _unity_timer (UnityTimer): A timer object for synchronizing with the Unity simulation clock.

    """

    def _setup_env_for_training(self, reward_fnc: str, **kwargs):
        """
        Set up the environment for training.

        Args:
            reward_fnc (str): The name of the reward function to use.
            **kwargs: Additional keyword arguments.

        Returns:
            The result of calling the superclass's _setup_env_for_training method.

        """
        # Unity specific
        clock_topic = self.ns.simulation_ns("clock")
        clock_msg = rospy.wait_for_message(clock_topic, Clock, timeout=30)
        self._unity_timer = UnityTimer(
            self._step_size,
            rospy.Time(clock_msg.clock.secs, clock_msg.clock.nsecs),
            clock_topic,
        )
        return super()._setup_env_for_training(reward_fnc, **kwargs)

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
