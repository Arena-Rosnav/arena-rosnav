from typing import List, Tuple, Type, Union, Callable, Any

import gym
import rosnav_rl
from rl_utils.cfg import (
    GeneralCfg,
    MonitoringCfg,
    ProfilingCfg,
)
from rl_utils.envs.flatland_gymnasium_env import FlatlandEnv
from rl_utils.envs.unity import UnityEnv
from rl_utils.stable_baselines3.vec_wrapper import (
    DelayedSubprocVecEnv,
    ProfilingVecEnv,
    VecStatsRecorder,
)
from rl_utils.utils.constants import Simulator
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import DummyVecEnv, VecFrameStack
from stable_baselines3.common.vec_env.base_vec_env import VecEnv

from task_generator.shared import Namespace
from task_generator.utils import Utils

from .constants import SIMULATION_NAMESPACES


def load_vec_framestack(stack_size: int, env: VecEnv) -> VecEnv:
    """
    Load a vectorized environment with frame stacking.

    Args:
        stack_size (int): The number of frames to stack.
        env (VecEnv): The vectorized environment to wrap.

    Returns:
        VecEnv: The wrapped vectorized environment with frame stacking applied.
    """
    return VecFrameStack(env, n_stack=stack_size, channels_order="first")


def determine_env_class(simulator: Simulator) -> Union[gym.Env, gym.Wrapper]:
    """
    Determines the environment class based on the specified simulator.

    Args:
        simulator (Simulator): The simulator to use.

    Returns:
        Union[gym.Env, gym.Wrapper]: The environment class.
    """
    if simulator == Simulator.FLATLAND:
        return FlatlandEnv
    elif simulator == Simulator.UNITY:
        return UnityEnv
    else:
        raise RuntimeError(f"Simulator {simulator} is not supported.")


def _init_env_fnc(
    env_class: gym.Env,
    ns: Union[str, Namespace],
    space_manager: rosnav_rl.BaseSpaceManager,
    reward_function: rosnav_rl.RewardFunction,
    simulation_state_container: rosnav_rl.SimulationStateContainer,
    max_steps_per_episode: int,
    init_by_call: bool = False,
    obs_unit_kwargs: dict = None,
    task_generator_kwargs: dict = None,
    seed: int = 0,
    wrappers: List[Callable[[Tuple[Type[gym.Wrapper], Any]], gym.Wrapper]] = None,
) -> callable:

    def _init_env() -> Union[gym.Env, gym.Wrapper]:
        env = env_class(
            ns=ns,
            space_manager=space_manager,
            reward_function=reward_function,
            simulation_state_container=simulation_state_container,
            max_steps_per_episode=max_steps_per_episode,
            init_by_call=init_by_call,
            obs_unit_kwargs=obs_unit_kwargs,
            task_generator_kwargs=task_generator_kwargs,
            start_ros_node=False,
        )
        for wrapper in wrappers or []:
            env = wrapper(env)
        return env

    set_random_seed(seed)
    return _init_env


def sb3_wrap_env(
    train_env_fncs: List[callable],
    eval_env_fncs: List[callable],
    general_cfg: GeneralCfg,
    monitoring_cfg: MonitoringCfg,
    profiling_cfg: ProfilingCfg,
) -> Tuple[VecEnv, VecEnv]:
    """
    Wraps the training and evaluation environments with the necessary vectorized wrappers.

    Args:
        train_env_fncs (List[callable]): List of functions to initialize training environments.
        eval_env_fncs (List[callable]): List of functions to initialize evaluation environments.
        rl_agent (Rosnav_RL_Agent): The reinforcement learning agent.
        agent_cfg (AgentCfg): Configuration for the agent.
        general_cfg (GeneralCfg): General configuration.
        normalization_cfg (NormalizationCfg): Configuration for normalization.
        monitoring_cfg (MonitoringCfg): Configuration for monitoring.
        profiling_cfg (ProfilingCfg): Configuration for profiling.

    Returns:
        Tuple[VecEnv, VecEnv]: The wrapped training and evaluation environments.
    """

    def create_train_env():
        return (
            DelayedSubprocVecEnv(train_env_fncs, start_method="fork")
            if not general_cfg.debug_mode
            else DummyVecEnv(train_env_fncs)
        )

    def create_eval_env():
        return DummyVecEnv(eval_env_fncs)

    def apply_vec_stats_recorder(env: VecEnv) -> VecEnv:
        return (
            VecStatsRecorder(
                env,
                after_x_eps=monitoring_cfg.episode_logging.last_n_episodes,
                record_actions=monitoring_cfg.episode_logging.record_actions,
            )
            if monitoring_cfg.episode_logging is not None
            else env
        )

    def apply_profiling(env: VecEnv, enable_subscribers: bool = True) -> VecEnv:
        return (
            ProfilingVecEnv(
                env=env,
                profile_step=profiling_cfg.do_profile_step,
                profile_reset=profiling_cfg.do_profile_reset,
                per_call=profiling_cfg.per_call,
                log_file=profiling_cfg.log_file,
                enable_subscribers=enable_subscribers,
            )
            if profiling_cfg is not None
            else env
        )

    train_env = create_train_env()
    eval_env = create_eval_env()

    # train_env = apply_vec_framestack(train_env)
    # eval_env = apply_vec_framestack(eval_env)

    # train_env = apply_vec_normalize(train_env, is_training=True)
    # eval_env = apply_vec_normalize(eval_env, is_training=False)

    train_env = apply_vec_stats_recorder(train_env)
    eval_env = apply_vec_stats_recorder(eval_env)

    train_env = apply_profiling(train_env)
    eval_env = apply_profiling(eval_env, enable_subscribers=False)

    return train_env, eval_env


def make_envs(
    rl_agent: rosnav_rl.RL_Agent,
    simulation_state_container: rosnav_rl.SimulationStateContainer,
    n_envs: int,
    max_steps: int,
    init_env_by_call: bool,
    namespace_fn: callable,
    wrappers: List[Callable[[Tuple[Type[gym.Wrapper], Any]], gym.Wrapper]] = None,
) -> List[callable]:
    """
    Creates a list of environment initialization functions.

    This function generates callable functions that each initialize a gym environment
    for reinforcement learning training. The environments are configured with the
    specified RL agent's space manager and reward function.

    Args:
        rl_agent: The reinforcement learning agent containing space manager and reward function
        simulation_state_container: Container holding the state of the simulation
        n_envs: Number of environments to create
        max_steps: Maximum number of steps per episode for each environment
        init_env_by_call: If True, environments will be initialized when their function is called
        namespace_fn: Function that takes an index and returns a namespace for the environment
        wrappers: Optional list of gym wrappers to apply to each environment

    Returns:
        List of callables, each initializing a gym environment when called
    """

    def create_env_fnc(
        ns: Union[str, Namespace], max_steps: int, init_env_by_call: bool
    ) -> callable:
        return _init_env_fnc(
            env_class=determine_env_class(Utils.get_simulator()),
            ns=ns,
            space_manager=rl_agent.space_manager,
            reward_function=rl_agent.reward_function.copy(),
            simulation_state_container=simulation_state_container,
            max_steps_per_episode=max_steps,
            init_by_call=init_env_by_call,
            wrappers=wrappers,
        )

    def create_env_fncs(
        n_envs: int, ns_fn: callable, max_steps: int, init_env_by_call: bool
    ) -> List[callable]:
        return [
            create_env_fnc(
                ns=ns_fn(idx),
                max_steps=max_steps,
                init_env_by_call=init_env_by_call,
            )
            for idx in range(n_envs)
        ]

    return create_env_fncs(
        n_envs=n_envs,
        ns_fn=namespace_fn,
        max_steps=max_steps,
        init_env_by_call=init_env_by_call,
    )
