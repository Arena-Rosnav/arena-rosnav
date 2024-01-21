import time
from stable_baselines3.common.vec_env import VecEnvWrapper, VecEnv

import numpy as np

from rl_utils.utils.observation_collector.constants import DONE_REASONS
from stable_baselines3.common.vec_env.base_vec_env import VecEnvObs


class VecStatsRecorder(VecEnvWrapper):
    """
    A wrapper class that records statistics of a vectorized environment.

    Args:
        venv (VecEnv): The vectorized environment to wrap.
        verbose (bool, optional): Whether to print the statistics or not. Defaults to False.
        x (int, optional): The frequency of printing the statistics. Defaults to 100.
    """

    def __init__(self, venv: VecEnv, verbose: bool = False, after_x_eps: int = 100):
        super(VecStatsRecorder, self).__init__(venv)

        assert after_x_eps > 0, "'after_x_eps' must be positive"

        self.verbose = verbose
        self.after_x_eps = after_x_eps
        self.num_envs = venv.num_envs

        self.num_steps = 0
        self.num_episodes = 0

        self.reset_stats()

    def reset_stats(self):
        """
        Reset the statistics.
        """
        self.step_times = []
        self.cum_rewards = np.array([0.0] * self.num_envs)
        self.episode_returns = []
        self.episode_lengths = []
        self.done_reasons = {done_reason.name: 0 for done_reason in DONE_REASONS}

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

        self.cum_rewards += rewards

        for idx, done in enumerate(dones):
            if done:
                self.episode_returns.append(self.cum_rewards[idx])
                self.cum_rewards[idx] = 0.0

                self.episode_lengths.append(infos[idx]["episode_length"])
                self.done_reasons[infos[idx]["done_reason"]] += 1

                self.num_episodes += 1

        self.num_steps += 1

        if (
            self.verbose
            and self.num_episodes % self.after_x_eps == 0
            and self.num_episodes > 0
        ):
            self.print_stats()
            self.reset_stats()

        return obs, rewards, dones, infos

    def print_stats(self):
        """
        Print the recorded statistics.
        """
        if len(self.episode_returns) == 0 or len(self.episode_lengths) == 0:
            return
        # Use print function to add line separators
        print("-" * 40, sep="", end="\n")  # Print 40 dashes as a line separator
        print(f"Episode {self.num_episodes} / Step {self.num_steps}:")
        print(
            f"Average step time: {sum(self.step_times) / self.after_x_eps:.4f} seconds"
        )
        print(
            f"Average episode return: {sum(self.episode_returns) / self.after_x_eps:.3f} pts"
        )
        print(
            f"Mean episode length: {sum(self.episode_lengths) / self.after_x_eps:.1f} steps"
        )
        print(f"Done reasons: {self.done_reasons}")
        print("-" * 40, sep="", end="\n")  # Print another line separator

    def reset(self) -> VecEnvObs:
        return self.venv.reset()  # pytype:disable=annotation-type-mismatch
