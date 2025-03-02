import time

import numpy as np
from rosnav_rl.observations import LastActionCollector, DONE_REASONS
from stable_baselines3.common.vec_env import VecEnv, VecEnvWrapper
from stable_baselines3.common.vec_env.base_vec_env import VecEnvObs
import rospy
import wandb

BATCHED_ZERO_ACTION = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)


class VecStatsRecorder(VecEnvWrapper):
    """
    A wrapper class that records statistics of a vectorized environment.

    Args:
        venv (VecEnv): The vectorized environment to wrap.
        x (int, optional): The frequency of printing the statistics. Defaults to 100.
    """

    def __init__(
        self,
        venv: VecEnv,
        after_x_eps: int = 100,
        record_actions: bool = False,
        is_action_normalized: bool = True,
        *args,
        **kwargs,
    ):
        super(VecStatsRecorder, self).__init__(venv)

        assert after_x_eps > 0, "'after_x_eps' must be positive"

        self.after_x_eps = after_x_eps
        self.num_envs = venv.num_envs
        self.is_action_normalized = is_action_normalized

        self.num_steps = 0
        self.num_episodes = 0

        self.last_print_at_step = 0  # to calculate steps since last print

        np.set_printoptions(formatter={"float": "{:.2f}".format})
        self._record_actions = record_actions
        if self._record_actions:
            self.get_action_ranges()
        self.reset_stats()

    def get_action_ranges(self):
        linear_range = rospy.get_param("/actions/continuous/linear_range")
        angular_range = rospy.get_param("/actions/continuous/angular_range")
        translation_range = rospy.get_param(
            "/actions/continuous/translation_range", [0, 0]
        )
        self._action_min = np.array(
            [linear_range[0], translation_range[0], angular_range[0]]
        )
        self._action_max = np.array(
            [linear_range[1], translation_range[1], angular_range[1]]
        )

    def reset_stats(self):
        """
        Reset the statistics.
        """
        self.last_print_at_step = self.num_steps
        self.step_times = []
        self.cum_rewards = np.array([0.0] * self.num_envs)
        self.episode_returns = []
        self.episode_lengths = []
        self.done_reasons = {done_reason.name: 0 for done_reason in DONE_REASONS}
        self.actions = np.array([0, 0, 0], dtype=np.float32)

    def step_wait(self):
        """
        Perform a step in the wrapped environment and record the statistics.

        Returns:
            tuple: A tuple containing the observations, rewards, dones, and infos.
        """
        start_time = time.time()
        obs, rewards, dones, infos = self.venv.step_wait()
        end_time = time.time()

        self.step_times.append(end_time - start_time)

        if self._record_actions:
            mean_actions = (
                obs[LastActionCollector.name.upper()][:, :]
                if LastActionCollector.name.upper() in obs
                else BATCHED_ZERO_ACTION
            )
            self.actions += np.mean(mean_actions, axis=0)
        self.cum_rewards += rewards

        for idx, done in enumerate(dones):
            if done:
                self.episode_returns.append(self.cum_rewards[idx])
                self.cum_rewards[idx] = 0.0

                self.episode_lengths.append(infos[idx]["episode_length"])
                self.done_reasons[infos[idx]["done_reason"].name] += 1

                self.num_episodes += 1

        self.num_steps += 1

        if self.num_episodes % self.after_x_eps == 0 and self.num_episodes > 0:
            self.print_stats()
            self.reset_stats()

        return obs, rewards, dones, infos

    def print_stats(self):
        """
        Print the recorded statistics.
        """
        if len(self.episode_returns) == 0 or len(self.episode_lengths) == 0:
            return

        if self._record_actions:
            avg_actions = (
                reverse_max_abs_scaling(
                    self.actions / (self.num_steps - self.last_print_at_step),
                    min_value=self._action_min,
                    max_value=self._action_max,
                )
                if self.is_action_normalized
                else self.actions / self.num_steps
            )

        avg_step_time = sum(self.step_times) / len(self.step_times)
        avg_episode_return = sum(self.episode_returns) / len(self.episode_returns)
        avg_episode_length = sum(self.episode_lengths) / len(self.episode_lengths)

        print("-" * 40, sep="", end="\n")  # Print 40 dashes as a line separator
        print(f"Episode {self.num_episodes} / Step {self.num_steps}:")
        if self._record_actions:
            print(f"Average actions: {avg_actions} (linear, transversal, angular)")
        print(f"Average step time: {avg_step_time:.4f} seconds")
        print(f"Average episode return: {avg_episode_return:.3f} pts")
        print(f"Mean episode length: {avg_episode_length:.1f} steps")
        print(f"Done reasons: {self.done_reasons}")
        print("-" * 40, sep="", end="\n")  # Print another line separator

        wandb.log(
            {
                "train_episode/step_time": avg_step_time,
                "train_episode/reward": avg_episode_return,
                "train_episode/length": avg_episode_length,
                "train_episode/success_rate": self.done_reasons[
                    DONE_REASONS.SUCCESS.name
                ]
                / len(self.episode_lengths),
                "train_episode/collision_rate": self.done_reasons[
                    DONE_REASONS.COLLISION.name
                ]
                / len(self.episode_lengths),
                "train_episode/timeout_rate": self.done_reasons[
                    DONE_REASONS.STEP_LIMIT.name
                ]
                / len(self.episode_lengths),
            }
        )

    def reset(self) -> VecEnvObs:
        return self.venv.reset()  # pytype:disable=annotation-type-mismatch


def reverse_max_abs_scaling(
    observation_arr: np.ndarray, min_value: np.ndarray, max_value: np.ndarray
):
    denominator = max_value - min_value
    denominator = np.where(denominator == 0, 1e-8, denominator)
    return (observation_arr + 1) * denominator / 2 + min_value
