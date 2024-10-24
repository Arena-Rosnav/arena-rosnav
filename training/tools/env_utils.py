import os
from typing import Union, Tuple, List

import gym
import rospy
from rl_utils.envs.flatland_gymnasium_env import FlatlandEnv
from rl_utils.envs.unity import UnityEnv
from rl_utils.stable_baselines3.vec_wrapper import (
    DelayedSubprocVecEnv,
    ProfilingVecEnv,
    VecStatsRecorder,
)
from rosnav_rl.cfg import AgentCfg
from rosnav_rl.spaces.observation_space.observation_space_manager import (
    ObservationSpaceManager,
)
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import DummyVecEnv, VecFrameStack, VecNormalize
from stable_baselines3.common.vec_env.base_vec_env import VecEnv
from task_generator.constants import Constants
from task_generator.shared import Namespace
from task_generator.utils import Utils

from .constants import SIMULATION_NAMESPACES, TRAINING_CONSTANTS

from rosnav_rl.rl_agent import RL_Agent as Rosnav_RL_Agent
from rl_utils.state_container import SimulationStateContainer
from rl_utils.cfg import (
    GeneralCfg,
    CallbacksCfg,
    NormalizationCfg,
    MonitoringCfg,
    ProfilingCfg,
)


def load_vec_normalize(
    env: VecEnv,
    normalization_cfg: NormalizationCfg,
    agent_name: str,
    resume: bool = False,
    checkpoint_name: str = "",
    is_training: bool = True,
) -> VecEnv:
    """
    Load or initialize VecNormalize wrapper for the environment.

    Args:
        env (VecEnv): The vectorized environment to wrap.
        normalization_cfg (NormalizationCfg): Configuration for normalization.
        resume (bool, optional): Whether to resume from a checkpoint. Defaults to False.
        checkpoint_name (str, optional): The name of the checkpoint to load. Defaults to "".
        is_training (bool, optional): Whether the environment is for training. Defaults to True.

    Returns:
        VecEnv: The wrapped vectorized environment with normalization applied.
    """

    def initialize_vec_normalize():
        return VecNormalize(env, training=is_training, **normalization_cfg.model_dump())

    if not resume:
        return initialize_vec_normalize()

    vec_norm_path = TRAINING_CONSTANTS.PATHS.VEC_NORMALIZE(agent_name, checkpoint_name)
    if os.path.isfile(vec_norm_path):
        env = VecNormalize.load(load_path=vec_norm_path, venv=env)
        rospy.loginfo("Successfully loaded VecNormalize object from pickle file.")
    else:
        rospy.logwarn(
            f"Could not load VecNormalize object from pickle file {vec_norm_path}. "
            "Initializing a new VecNormalize object."
        )
        env = initialize_vec_normalize()

    return env


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


def _init_env_fnc(
    ns: Union[str, Namespace],
    rl_agent: Rosnav_RL_Agent,
    simulation_state_container: SimulationStateContainer,
    max_steps_per_episode: int,
    init_by_call: bool = False,
    obs_unit_kwargs: dict = None,
    task_generator_kwargs: dict = None,
    seed: int = 0,
) -> callable:
    """
    Initializes the environment function for the specified simulator.

    Args:
        ns (Union[str, Namespace]): Namespace or string identifier for the environment.
        rl_agent (Rosnav_RL_Agent): The reinforcement learning agent.
        simulation_state_container (SimulationStateContainer): Container for the simulation state.
        max_steps_per_episode (int): Maximum number of steps per episode.
        init_by_call (bool, optional): Flag to initialize by call. Defaults to False.
        obs_unit_kwargs (dict, optional): Keyword arguments for observation unit. Defaults to None.
        task_generator_kwargs (dict, optional): Keyword arguments for task generator. Defaults to None.
        seed (int, optional): Random seed for initialization. Defaults to 0.

    Returns:
        callable: A function that initializes and returns the environment instance.

    Raises:
        RuntimeError: If the simulator is not supported.
    """
    sim = Utils.get_simulator()
    env_cls = {
        Constants.Simulator.UNITY: UnityEnv,
        Constants.Simulator.FLATLAND: FlatlandEnv,
    }.get(sim)

    if env_cls is None:
        raise RuntimeError(
            f"Training only supports simulators Arena Unity and Flatland but got {sim}"
        )

    def _init_env() -> Union[gym.Env, gym.Wrapper]:
        return env_cls(
            ns=ns,
            space_manager=rl_agent.space_manager,
            reward_function=rl_agent.reward_function,
            simulation_state_container=simulation_state_container,
            max_steps_per_episode=max_steps_per_episode,
            init_by_call=init_by_call,
            obs_unit_kwargs=obs_unit_kwargs,
            task_generator_kwargs=task_generator_kwargs,
        )

    set_random_seed(seed)
    return _init_env


def sb3_wrap_env(
    train_env_fncs: List[callable],
    eval_env_fncs: List[callable],
    rl_agent: Rosnav_RL_Agent,
    agent_cfg: AgentCfg,
    general_cfg: GeneralCfg,
    normalization_cfg: NormalizationCfg,
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

    def apply_vec_framestack(env: VecEnv) -> VecEnv:
        return (
            load_vec_framestack(rl_agent.model.stack_size, env)
            if rl_agent.model.stack_size > 1
            else env
        )

    def apply_vec_normalize(env: VecEnv, is_training: bool) -> VecEnv:
        return (
            load_vec_normalize(
                env=env,
                normalization_cfg=normalization_cfg,
                agent_name=agent_cfg.name,
                resume=bool(agent_cfg.policy.resume),
                checkpoint_name=agent_cfg.policy.checkpoint,
                is_training=is_training,
            )
            if normalization_cfg is not None
            else env
        )

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

    train_env = apply_vec_framestack(train_env)
    eval_env = apply_vec_framestack(eval_env)

    train_env = apply_vec_normalize(train_env, is_training=True)
    eval_env = apply_vec_normalize(eval_env, is_training=False)

    train_env = apply_vec_stats_recorder(train_env)
    eval_env = apply_vec_stats_recorder(eval_env)

    train_env = apply_profiling(train_env)
    eval_env = apply_profiling(eval_env, enable_subscribers=False)

    return train_env, eval_env


def make_envs(
    rl_agent: Rosnav_RL_Agent,
    simulation_state_container: SimulationStateContainer,
    agent_cfg: AgentCfg,
    general_cfg: GeneralCfg,
    callback_cfg: CallbacksCfg,
    normalization_cfg: NormalizationCfg,
    monitoring_cfg: MonitoringCfg,
    profiling_cfg: ProfilingCfg,
) -> Tuple[VecEnv, VecEnv]:
    """
    Create and wrap training and evaluation environments.

    Args:
        rl_agent (Rosnav_RL_Agent): The reinforcement learning agent.
        simulation_state_container (SimulationStateContainer): Container for the simulation state.
        agent_cfg (AgentCfg): Configuration for the agent.
        general_cfg (GeneralCfg): General configuration.
        callback_cfg (CallbacksCfg): Configuration for callbacks.
        normalization_cfg (NormalizationCfg): Configuration for normalization.
        monitoring_cfg (MonitoringCfg): Configuration for monitoring.
        profiling_cfg (ProfilingCfg): Configuration for profiling.

    Returns:
        Tuple[VecEnv, VecEnv]: The wrapped training and evaluation environments.
    """

    def create_env_fnc(ns: Union[str, Namespace], max_steps: int) -> callable:
        return _init_env_fnc(
            ns=ns,
            rl_agent=rl_agent,
            simulation_state_container=simulation_state_container,
            max_steps_per_episode=max_steps,
            init_by_call=not general_cfg.debug_mode,
        )

    train_env_fncs = [
        create_env_fnc(
            SIMULATION_NAMESPACES.TRAIN_NS(idx), general_cfg.max_num_moves_per_eps
        )
        for idx in range(general_cfg.n_envs)
    ]

    eval_env_fncs = [
        create_env_fnc(
            SIMULATION_NAMESPACES.EVAL_NS,
            callback_cfg.periodic_evaluation.max_num_moves_per_eps,
        )
    ]

    return sb3_wrap_env(
        train_env_fncs=train_env_fncs,
        eval_env_fncs=eval_env_fncs,
        rl_agent=rl_agent,
        agent_cfg=agent_cfg,
        general_cfg=general_cfg,
        normalization_cfg=normalization_cfg,
        monitoring_cfg=monitoring_cfg,
        profiling_cfg=profiling_cfg,
    )
