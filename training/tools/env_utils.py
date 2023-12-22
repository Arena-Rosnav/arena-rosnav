import os
from typing import Tuple, Union

import gym
import rospy

# from rl_utils.envs.flatland_gym_env import (
#     FlatlandEnv,
# )
from rl_utils.envs.flatland_gymnasium_env import FlatlandEnv
from rosnav.model.base_agent import BaseAgent
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
    VecFrameStack,
    VecNormalize,
)
from stable_baselines3.common.vec_env.base_vec_env import VecEnv


def make_envs(
    with_ns: bool,
    rank: int,
    config: dict,
    agent_description: BaseAgent,
    seed: int = 0,
    paths: dict = None,
    train: bool = True,
):
    """
    Utility function for multiprocessed env

    :param with_ns: (bool) if the system was initialized with namespaces
    :param rank: (int) index of the subprocess
    :param config: (dict) hyperparameters of agent to be trained
    :param seed: (int) the inital seed for RNG
    :param paths: (dict) script relevant paths
    :param train: (bool) to differentiate between train and eval env
    :param args: (Namespace) program arguments
    :return: (Callable)
    """

    def _init() -> Union[gym.Env, gym.Wrapper]:
        robot_model = rospy.get_param("model")
        train_ns = f"/sim_{rank + 1}/{robot_model}" if with_ns else ""
        eval_ns = f"/eval_sim/{robot_model}" if with_ns else ""

        curriculum_config = config["callbacks"]["training_curriculum"]
        log_config = config["monitoring"]["cmd_line_logging"]

        if train:
            # train env
            env = FlatlandEnv(
                ns=train_ns,
                agent_description=agent_description,
                reward_fnc=config["rl_agent"]["reward_fnc"],
                max_steps_per_episode=config["max_num_moves_per_eps"],
                verbose=log_config["episode_statistics"]["enabled"],
                log_last_n_eps=log_config["episode_statistics"]["last_n_eps"],
                starting_stage=curriculum_config["curr_stage"],
                curriculum_path=paths["curriculum"],
            )
        else:
            # eval env
            env = Monitor(
                FlatlandEnv(
                    ns=eval_ns,
                    agent_description=agent_description,
                    reward_fnc=config["rl_agent"]["reward_fnc"],
                    max_steps_per_episode=config["callbacks"]["periodic_eval"][
                        "max_num_moves_per_eps"
                    ],
                    verbose=log_config["episode_statistics"]["enabled"],
                    log_last_n_eps=log_config["episode_statistics"]["last_n_eps"],
                    starting_stage=curriculum_config["curr_stage"],
                    curriculum_path=paths["curriculum"],
                ),
                paths["eval"],
                info_keywords=("done_reason", "is_success"),
            )
        # env.seed(seed + rank)
        return env

    set_random_seed(seed)
    return _init


def load_vec_normalize(config: dict, paths: dict, env: VecEnv, eval_env: VecEnv):
    if config["rl_agent"]["normalize"]["enabled"]:
        load_path = os.path.join(paths["model"], "vec_normalize.pkl")
        if os.path.isfile(load_path):
            env = VecNormalize.load(load_path=load_path, venv=env)
            eval_env = VecNormalize.load(load_path=load_path, venv=eval_env)
            print("Succesfully loaded VecNormalize object from pickle file..")
        elif not config["rl_agent"]["resume"]:
            # New agent so init new VecNormalize object
            normalization_conf = config["rl_agent"]["normalize"]["settings"]
            env = VecNormalize(env, training=True, **normalization_conf)
            eval_env = VecNormalize(eval_env, training=False, **normalization_conf)
        else:
            raise ValueError("No VecNormalize object found..")
    return env, eval_env


def load_vec_framestack(config: dict, env: VecEnv, eval_env: VecEnv):
    fs_cfg = config["rl_agent"]["frame_stacking"]
    if fs_cfg["enabled"]:
        env = VecFrameStack(env, n_stack=fs_cfg["stack_size"], channels_order="first")
        eval_env = VecFrameStack(
            eval_env, n_stack=fs_cfg["stack_size"], channels_order="first"
        )
    return env, eval_env


def init_envs(
    agent_description: BaseAgent,
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
                    with_ns=ns_for_nodes,
                    rank=i,
                    config=config,
                    paths=paths,
                    agent_description=agent_description,
                )
                for i in range(config["n_envs"])
            ],
            start_method="fork",
        )
    else:
        train_env = DummyVecEnv(
            [
                make_envs(
                    with_ns=ns_for_nodes,
                    rank=i,
                    config=config,
                    paths=paths,
                    agent_description=agent_description,
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
                    with_ns=ns_for_nodes,
                    rank=0,
                    config=config,
                    paths=paths,
                    agent_description=agent_description,
                    train=False,
                )
            ]
        )
    else:
        eval_env = train_env

    train_env, eval_env = load_vec_framestack(config, train_env, eval_env)
    return load_vec_normalize(config, paths, train_env, eval_env)
