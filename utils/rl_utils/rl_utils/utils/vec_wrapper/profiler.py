import pyinstrument
from stable_baselines3.common.vec_env import VecEnvWrapper


class ProfilingVecEnv(VecEnvWrapper):
    """
    A vectorized environment wrapper that adds profiling capabilities.

    Args:
        env (VecEnv): The vectorized environment to wrap.
        profile_step (bool): Whether to profile the `step` method. Default is True.
        profile_reset (bool): Whether to profile the `reset` method. Default is True.
        per_call (bool): Whether to reset the profiler after each call. Default is False.
    """

    def __init__(
        self,
        env,
        profile_step: bool = True,
        profile_reset: bool = True,
        per_call: bool = False,
    ):
        super().__init__(env)
        self._step_profiler = pyinstrument.Profiler()
        self._reset_profiler = pyinstrument.Profiler()

        self._profile_method_step = profile_step
        self._profile_method_reset = profile_reset

        self._per_call = per_call

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
            print(self._step_profiler.output_text(unicode=True, color=True))

        if self._per_call:
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
            print(self._reset_profiler.output_text(unicode=True, color=True))

        if self._per_call:
            self._reset_profiler.reset()

        return observations
