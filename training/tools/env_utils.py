from typing import Union, Tuple

import gym
import os

from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import VecNormalize, SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.vec_env.base_vec_env import VecEnv

from rl_utils.envs.flatland_gym_env import (
    FlatlandEnv,
)


def make_envs(
    with_ns: bool,
    rank: int,
    config: dict,
    seed: int = 0,
    PATHS: dict = None,
    train: bool = True,
):
    """
    Utility function for multiprocessed env

    :param with_ns: (bool) if the system was initialized with namespaces
    :param rank: (int) index of the subprocess
    :param config: (dict) hyperparameters of agent to be trained
    :param seed: (int) the inital seed for RNG
    :param PATHS: (dict) script relevant paths
    :param train: (bool) to differentiate between train and eval env
    :param args: (Namespace) program arguments
    :return: (Callable)
    """

    def _init() -> Union[gym.Env, gym.Wrapper]:
        train_ns = f"sim_{rank + 1}" if with_ns else ""
        eval_ns = "eval_sim" if with_ns else ""

        curriculum_config = config["callbacks"]["training_curriculum"]
        if train:
            # train env
            env = FlatlandEnv(
                train_ns,
                config["rl_agent"]["reward_fnc"],
                config["rl_agent"]["discrete_action_space"],
                goal_radius=config["goal_radius"],
                max_steps_per_episode=config["max_num_moves_per_eps"],
                task_mode=config["task_mode"],
                curr_stage=curriculum_config["curr_stage"],
                PATHS=PATHS,
            )
        else:
            # eval env
            env = Monitor(
                FlatlandEnv(
                    eval_ns,
                    config["rl_agent"]["reward_fnc"],
                    config["rl_agent"]["discrete_action_space"],
                    goal_radius=config["goal_radius"],
                    max_steps_per_episode=config["max_num_moves_per_eps"],
                    task_mode=config["task_mode"],
                    curr_stage=curriculum_config["curr_stage"],
                    PATHS=PATHS,
                ),
                PATHS["eval"],
                info_keywords=("done_reason", "is_success"),
            )
        # env.seed(seed + rank)
        return env

    set_random_seed(seed)
    return _init


def load_vec_normalize(config: dict, PATHS: dict, env: VecEnv, eval_env: VecEnv):
    if config["rl_agent"]["normalize"]:
        load_path = os.path.join(PATHS["model"], "vec_normalize.pkl")
        if os.path.isfile(load_path):
            env = VecNormalize.load(load_path=load_path, venv=env)
            eval_env = VecNormalize.load(load_path=load_path, venv=eval_env)
            print("Succesfully loaded VecNormalize object from pickle file..")
        else:
            env = VecNormalize(
                env,
                training=True,
                norm_obs=True,
                norm_reward=False,
                clip_reward=15,
            )
            eval_env = VecNormalize(
                eval_env,
                training=True,
                norm_obs=True,
                norm_reward=False,
                clip_reward=15,
            )
    return env, eval_env


def init_envs(
    config: dict,
    paths: dict,
    ns_for_nodes: bool,
) -> Tuple[VecEnv, VecEnv]:
    # instantiate train environment
    # when debug run on one process only
    if not config["debug_mode"] and ns_for_nodes:
        train_env = SubprocVecEnv(
            [
                make_envs(
                    ns_for_nodes,
                    i,
                    config=config,
                    PATHS=paths,
                )
                for i in range(config["n_envs"])
            ],
            start_method="fork",
        )
    else:
        train_env = DummyVecEnv(
            [
                make_envs(
                    ns_for_nodes,
                    i,
                    config=config,
                    PATHS=paths,
                )
                for i in range(config["n_envs"])
            ]
        )

    # instantiate eval environment
    # take task_manager from first sim (currently evaluation only provided for single process)
    if ns_for_nodes:
        eval_env = DummyVecEnv(
            [
                make_envs(
                    ns_for_nodes,
                    0,
                    config=config,
                    PATHS=paths,
                    train=False,
                )
            ]
        )
    else:
        eval_env = train_env

    return load_vec_normalize(config, paths, train_env, eval_env)
