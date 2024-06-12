import rospy
from geometry_msgs.msg import Twist
from rl_utils.utils.arena_unity_utils.unity_timer import UnityTimer
from rl_utils.utils.observation_collector import DoneObservation
from rosgraph_msgs.msg import Clock

from .flatland_gymnasium_env import FlatlandEnv


class UnityEnv(FlatlandEnv):

    def _setup_env_for_training(self, reward_fnc: str, **kwargs):
        # Unity specific
        clock_topic = self.ns.simulation_ns("clock")
        clock_msg = rospy.wait_for_message(clock_topic, Clock, timeout=30)
        self._unity_timer = UnityTimer(
            self._step_size,
            rospy.Time(clock_msg.clock.secs, clock_msg.clock.nsecs),
            clock_topic,
        )
        return super()._setup_env_for_training(reward_fnc, **kwargs)

    def reset(self, seed=None, options=None):
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

        if self._is_train_mode:
            self.agent_action_pub.publish(Twist())
            self._unity_timer.wait_for_next_update()

        obs_dict = self.observation_collector.get_observations()
        obs_dict.update({DoneObservation.name: True})

        info_dict = {}
        return (
            self._encode_observation(obs_dict),
            info_dict,
        )

    def render(self):
        pass

    def close(self):
        """
        Close the environment.

        """
        rospy.loginfo("[Unity Env ns:" + self.ns + "]: Closing environment.")
        pass
