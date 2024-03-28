import os
import sys
from typing import Tuple, Union

import gym
import rospy
from rl_utils.envs.flatland_gymnasium_env import FlatlandEnv
from rl_utils.utils.vec_wrapper.delayed_subproc_vec_env import DelayedSubprocVecEnv
from rl_utils.utils.vec_wrapper.vec_stats_recorder import VecStatsRecorder
from rl_utils.utils.vec_wrapper.profiler import ProfilingVecEnv
from rosnav.model.base_agent import BaseAgent
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
    VecFrameStack,
    VecNormalize,
)
from stable_baselines3.common.vec_env.base_vec_env import VecEnv
from task_generator.shared import Namespace


def load_vec_normalize(config: dict, paths: dict, env: VecEnv, eval_env: VecEnv):
    """
    Loads and initializes VecNormalize object for the environment.

    Args:
        config (dict): Configuration dictionary.
        paths (dict): Paths dictionary.
        env (VecEnv): Training environment.
        eval_env (VecEnv): Evaluation environment.

    Returns:
        Tuple[VecEnv, VecEnv]: Tuple containing the loaded and initialized VecNormalize objects for the environment.
    """
    agent_name = config["agent_name"]
    checkpoint = config["rl_agent"]["checkpoint"]
    possible_agent_names = [
        checkpoint,
        "best_model",
        "last_model",
        f"{agent_name}",
        "model",
    ]

    if not config["rl_agent"]["resume"]:
        # New agent so init new VecNormalize object
        normalization_conf = config["rl_agent"]["normalize"]["settings"]
        env = VecNormalize(env, training=True, **normalization_conf)
        eval_env = VecNormalize(eval_env, training=False, **normalization_conf)
        return env, eval_env

    for name in possible_agent_names:
        load_path = os.path.join(paths["model"], f"vec_normalize_{name}.pkl")
        if os.path.isfile(load_path):
            env = VecNormalize.load(load_path=load_path, venv=env)
            eval_env = VecNormalize.load(load_path=load_path, venv=eval_env)
            rospy.loginfo("Succesfully loaded VecNormalize object from pickle file..")
    return env, eval_env


def load_vec_framestack(config: dict, env: VecEnv, eval_env: VecEnv):
    """
    Load vectorized framestack environment.

    Args:
        config (dict): Configuration dictionary.
        env (VecEnv): Training environment.
        eval_env (VecEnv): Evaluation environment.

    Returns:
        Tuple[VecEnv, VecEnv]: Tuple containing the modified training and evaluation environments.
    """
    fs_cfg = config["rl_agent"]["frame_stacking"]
    env = VecFrameStack(env, n_stack=fs_cfg["stack_size"], channels_order="first")
    eval_env = VecFrameStack(
        eval_env, n_stack=fs_cfg["stack_size"], channels_order="first"
    )
    return env, eval_env


def _init_env_fnc(
    ns: Union[str, Namespace],
    agent_description: BaseAgent,
    reward_fnc: str,
    max_steps_per_episode: int,
    seed: int = 0,
    trigger_init: bool = False,
    obs_unit_kwargs: dict = None,
    reward_fnc_kwargs: dict = None,
    task_generator_kwargs: dict = None,
):
    """
    Initialize the environment function.

    Args:
        ns (Union[str, Namespace]): The namespace of the environment.
        agent_description (BaseAgent): The agent description.
        reward_fnc (str): The reward function.
        max_steps_per_episode (int): The maximum number of steps per episode.
        seed (int, optional): The random seed. Defaults to 0.

    Returns:
        Union[gym.Env, gym.Wrapper]: The initialized environment.
    """
    reward_fnc_kwargs = reward_fnc_kwargs or {}

    def _init() -> Union[gym.Env, gym.Wrapper]:
        return FlatlandEnv(
            ns=ns,
            agent_description=agent_description,
            reward_fnc=reward_fnc,
            max_steps_per_episode=max_steps_per_episode,
            trigger_init=trigger_init,
            obs_unit_kwargs=obs_unit_kwargs,
            reward_fnc_kwargs=reward_fnc_kwargs,
            task_generator_kwargs=task_generator_kwargs,
        )

    set_random_seed(seed)
    return _init


def make_envs(
    agent_description: BaseAgent,
    config: dict,
    paths: dict,
):
    """
    Create training and evaluation environments.

    Args:
        agent_description (BaseAgent): The agent description.
        config (dict): Configuration parameters.
        paths (dict): Paths for loading and saving data.

    Returns:
        tuple: A tuple containing the training environment and evaluation environment.
    """
    SIM_PREFIX = "sim_"
    EVAL_PREFIX = "eval_sim"
    train_ns = (
        lambda idx: f"/{SIM_PREFIX}{idx + 1}/{SIM_PREFIX}{idx + 1}_{rospy.get_param('model')}"
    )
    eval_ns = f"/{EVAL_PREFIX}/{EVAL_PREFIX}_{rospy.get_param('model')}"

    obs_unit_kwargs = {"subgoal_mode": config["rl_agent"]["subgoal_mode"]}

    train_env_fncs = [
        _init_env_fnc(
            ns=train_ns(idx),
            agent_description=agent_description,
            reward_fnc=config["rl_agent"]["reward_fnc"],
            max_steps_per_episode=config["max_num_moves_per_eps"],
            trigger_init=True if not config["debug_mode"] else False,
            obs_unit_kwargs=obs_unit_kwargs,
            reward_fnc_kwargs=config["rl_agent"]["reward_fnc_kwargs"],
        )
        for idx in range(config["n_envs"])
    ]

    eval_env_fncs = [
        _init_env_fnc(
            ns=eval_ns,
            agent_description=agent_description,
            reward_fnc=config["rl_agent"]["reward_fnc"],
            max_steps_per_episode=config["callbacks"]["periodic_eval"][
                "max_num_moves_per_eps"
            ],
            trigger_init=False,
            obs_unit_kwargs=obs_unit_kwargs,
            reward_fnc_kwargs=config["rl_agent"]["reward_fnc_kwargs"],
        )
    ]

    # vectorize environments
    train_env = (
        DelayedSubprocVecEnv(train_env_fncs, start_method="fork")
        if not config["debug_mode"]
        else DummyVecEnv(train_env_fncs)
    )
    eval_env = DummyVecEnv(eval_env_fncs)

    observation_manager = eval_env.envs[0].model_space_encoder.observation_space_manager

    # load vec wrappers
    if config["rl_agent"]["frame_stacking"]["enabled"]:
        train_env, eval_env = load_vec_framestack(config, train_env, eval_env)

    # load vec normalize
    if config["rl_agent"]["normalize"]["enabled"]:
        train_env, eval_env = load_vec_normalize(config, paths, train_env, eval_env)

    # wrap env with statistics wrapper
    cmd_logging_cfg = config["monitoring"]["cmd_line_logging"]["episode_statistics"]
    verbose = cmd_logging_cfg["enabled"]

    train_env = VecStatsRecorder(
        train_env,
        verbose=verbose,
        after_x_eps=cmd_logging_cfg["last_n_eps"],
    )
    eval_env = VecStatsRecorder(
        eval_env,
        verbose=verbose,
        after_x_eps=config["callbacks"]["periodic_eval"]["n_eval_episodes"],
    )

    # train_env = ProfilingVecEnv(
    #     train_env, profile_step=True, profile_reset=True, per_call=True
    # )
    return train_env, eval_env, observation_manager
