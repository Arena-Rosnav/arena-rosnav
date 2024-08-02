import pyinstrument
import rospy
from stable_baselines3.common.vec_env import VecEnvWrapper
from std_msgs.msg import Bool


class ProfilingVecEnv(VecEnvWrapper):
    """
    A vectorized environment wrapper that adds profiling capabilities.

    Args:
        env (VecEnv): The vectorized environment to wrap.
        profile_step (bool): Whether to profile the `step` method. Default is False.
        profile_reset (bool): Whether to profile the `reset` method. Default is False.
        per_call (bool): Whether to reset the profiler after each call. Default is False.
        print_stats (bool): Whether to print the profiling stats. Default is True.
        log_file (str): Path to the file where profiling stats should be logged. Default is None (no logging).
    """

    def __init__(
        self,
        env,
        profile_step: bool = False,
        profile_reset: bool = False,
        per_call: bool = True,
        print_stats: bool = True,
        log_file: str = None,
        enable_subscribers: bool = True,
    ):
        super().__init__(env)
        self._step_profiler = pyinstrument.Profiler()
        self._reset_profiler = pyinstrument.Profiler()

        self._profile_method_step = profile_step
        self._profile_method_reset = profile_reset

        self._per_call = per_call
        self._print_stats = print_stats
        self._log_file = log_file

        if enable_subscribers:
            # Set up subscribers
            rospy.Subscriber(
                "/profiler/profile_step", Bool, self._profile_step_callback
            )
            rospy.Subscriber(
                "/profiler/profile_reset", Bool, self._profile_reset_callback
            )

    def _profile_step_callback(self, msg):
        self._profile_method_step = msg.data
        rospy.loginfo(f"Profile step set to: {self._profile_method_step}")

    def _profile_reset_callback(self, msg):
        self._profile_method_reset = msg.data
        rospy.loginfo(f"Profile reset set to: {self._profile_method_reset}")

    def _output_stats(self, profiler: pyinstrument.Profiler, method_name: str):
        if self._print_stats:
            rospy.loginfo(f"Profiling stats for {method_name}:")
            rospy.loginfo(profiler.output_text(unicode=True, color=True))

        if self._log_file:
            with open(self._log_file, "a") as f:
                f.write(f"\nProfiling stats for {method_name}:\n")
                f.write(profiler.output_text(unicode=True, color=False))

    def step_wait(self):
        """
        Perform a step in the wrapped environment.

        Returns:
            tuple: A tuple containing the observations, rewards, dones, and infos.
        """
        if self._profile_method_step:
            self._step_profiler.start()

        obs, rewards, dones, infos = self.venv.step_wait()

        if self._profile_method_step:
            self._step_profiler.stop()
            self._output_stats(self._step_profiler, "step_wait")

        if (
            self._profile_method_step
            and self._per_call
            and self._step_profiler.is_running
        ):
            self._step_profiler.reset()

        return obs, rewards, dones, infos

    def reset(self):
        """
        Reset the wrapped environment.

        Returns:
            object: The initial observations.
        """
        if self._profile_method_reset:
            self._reset_profiler.start()

        observations = self.venv.reset()

        if self._profile_method_reset:
            self._reset_profiler.stop()
            self._output_stats(self._reset_profiler, "reset")

        if (
            self._profile_method_reset
            and self._per_call
            and self._reset_profiler.is_running
        ):
            self._reset_profiler.reset()

        return observations
