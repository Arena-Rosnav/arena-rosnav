from platform import architecture
from typing import Union, Tuple, Type
from datetime import datetime as dt

import argparse
import gym
import json
import os, rospy
from rosnav.model.base_agent import BaseAgent
import rosnode
import rospkg
import time
import warnings

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    EvalCallback,
    StopTrainingOnRewardThreshold,
)
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.utils import set_random_seed


from rl_utils.envs.flatland_gym_env import (
    FlatlandEnv,
)


def initialize_config(
    PATHS: dict,
    config: dict,
    n_envs: int = 1,
    debug_mode: bool = False,
) -> dict:
    """
    Write hyperparameters to json file in case agent is new otherwise load existing hyperparameters

    :param PATHS: dictionary containing model specific paths
    :param load_target: unique agent name (when calling --load)
    :param config_name: name of the hyperparameter file in /configs/hyperparameters
    :param n_envs: number of envs
    """
    import rosnav.model.custom_policy
    import rosnav.model.custom_sb3_policy

    # dynamically adapt n_steps according to batch size and n envs
    # then update .json
    check_batch_size(
        n_envs,
        config["rl_agent"]["ppo"]["batch_size"],
        config["rl_agent"]["ppo"]["m_batch_size"],
    )
    config["rl_agent"]["ppo"]["n_steps"] = int(
        config["rl_agent"]["ppo"]["batch_size"] / n_envs
    )
    config["rl_agent"]["space_encoder"] = rospy.get_param(
        "space_encoder", "RobotSpecificEncoder"
    )

    if not debug_mode:
        write_config_yaml(config, PATHS)
    print_hyperparameters(config["rl_agent"]["ppo"])

    return config


def write_config_yaml(config: dict, PATHS: dict) -> None:
    """
    Write training_config.yaml to agent directory

    :param hyperparams: dict containing model specific hyperparameters
    :param PATHS: dictionary containing model specific paths
    """
    with open(PATHS["config"], "w") as outfile:
        yaml.dump(config, outfile, default_flow_style=False)


def write_hyperparameters_json(hyperparams: dict, PATHS: dict) -> None:
    """
    Write hyperparameters.json to agent directory

    :param hyperparams: dict containing model specific hyperparameters
    :param PATHS: dictionary containing model specific paths
    """
    doc_location = os.path.join(PATHS.get("model"), "hyperparameters.json")

    with open(doc_location, "w", encoding="utf-8") as target:
        json.dump(hyperparams, target, ensure_ascii=False, indent=4)


# LEGACY CODE
# def load_hyperparameters_json(
#     PATHS: dict, from_scratch: bool = False, config_name: str = "default"
# ) -> dict:
#     """
#     Load hyperparameters from model directory when loading - when training from scratch
#     load from ../configs/hyperparameters

#     :param PATHS: dictionary containing model specific paths
#     :param from_scatch: if training from scratch
#     :param config_name: file name of json file when training from scratch
#     """
#     if from_scratch:
#         doc_location = os.path.join(PATHS["hyperparams"])
#     else:
#         doc_location = os.path.join(PATHS.get("model"), "hyperparameters.json")

#     if os.path.isfile(doc_location):
#         with open(doc_location, "r") as file:
#             hyperparams = json.load(file)
#         check_hyperparam_format(loaded_hyperparams=hyperparams, PATHS=PATHS)
#         return hyperparams
#     else:
#         if from_scratch:
#             raise FileNotFoundError(
#                 f"""Found no '{config_name}' in {PATHS.get("hyperparams")}"""
#             )
#         else:
#             raise FileNotFoundError(
#                 f"""Found no 'hyperparameters.json' in {PATHS.get("model")}"""
#             )


def print_hyperparameters(hyperparams: dict) -> None:
    print("\n--------------------------------")
    print("         HYPERPARAMETERS         \n")
    for param, param_val in hyperparams.items():
        print("{:30s}{:<10s}".format(f"{param}:", str(param_val)))
    print("--------------------------------\n\n")


# LEGACY CODE
#
# def check_hyperparam_format(loaded_hyperparams: dict, PATHS: dict) -> None:
#     if set(HYPERPARAM_KEYS.keys()) != set(loaded_hyperparams.keys()):
#         missing_keys = set(HYPERPARAM_KEYS.keys()).difference(
#             set(loaded_hyperparams.keys())
#         )

#         redundant_keys = set(loaded_hyperparams.keys()).difference(
#             set(HYPERPARAM_KEYS.keys())
#         )

#         raise AssertionError(
#             f"unmatching keys, following keys missing: {missing_keys} \n"
#             f"following keys unused: {redundant_keys}"
#         )
#     if not isinstance(loaded_hyperparams["discrete_action_space"], bool):
#         raise TypeError("Parameter 'discrete_action_space' not of type bool")
#     if loaded_hyperparams["task_mode"] not in ["custom", "random", "staged"]:
#         raise TypeError("Parameter 'task_mode' has unknown value")


def update_hyperparam_model(model: PPO, PATHS: dict, params: dict) -> None:
    """
    Updates parameter of loaded PPO agent when it was manually changed in the configs yaml.

    :param model(object, PPO): loaded PPO agent
    :param PATHS: program relevant paths
    :param params: dictionary containing loaded hyperparams
    :param n_envs: number of parallel environments
    """
    ppo_params = params["rl_agent"]["ppo"]

    model.batch_size = ppo_params["batch_size"]
    model.gamma = ppo_params["gamma"]
    model.n_steps = ppo_params["n_steps"]
    model.ent_coef = ppo_params["ent_coef"]
    model.learning_rate = ppo_params["learning_rate"]
    model.vf_coef = ppo_params["vf_coef"]
    model.max_grad_norm = ppo_params["max_grad_norm"]
    model.gae_lambda = ppo_params["gae_lambda"]
    model.n_epochs = ppo_params["n_epochs"]
    """
    if model.clip_range != params['clip_range']:
        model.clip_range = params['clip_range']
    """
    if model.n_envs != params["n_envs"]:
        model.update_n_envs()
        model.rollout_buffer.buffer_size = ppo_params["n_steps"]
        model.tensorboard_log = PATHS["tb"]


def init_wandb(model: PPO):
    import wandb

    logger_config = {
        "learning_rate": model.learning_rate,
        "n_steps": model.n_steps,
        "batch_size": model.batch_size,
        "n_epochs": model.n_epochs,
        "gamma": model.gamma,
        "gae_lambda": model.gae_lambda,
        "ent_coef": model.ent_coef,
        "vf_coef": model.vf_coef,
        "max_grad_norm": model.max_grad_norm,
        "use_sde": model.use_sde,
        "sde_sample_freq": model.sde_sample_freq,
        "clip_range": model.clip_range,
        "clip_range_vf": model.clip_range_vf,
        "target_kl": model.target_kl,
    }

    wandb.init(
        project="Arena-RL",
        entity=None,
        sync_tensorboard=True,
        monitor_gym=True,
        save_code=True,
        config=logger_config,
    )


def check_batch_size(n_envs: int, batch_size: int, mn_batch_size: int) -> None:
    assert (
        batch_size > mn_batch_size
    ), f"Mini batch size {mn_batch_size} is bigger than batch size {batch_size}"

    assert (
        batch_size % mn_batch_size == 0
    ), f"Batch size {batch_size} isn't divisible by mini batch size {mn_batch_size}"

    assert (
        batch_size % n_envs == 0
    ), f"Batch size {batch_size} isn't divisible by n_envs {n_envs}"

    assert (
        batch_size % mn_batch_size == 0
    ), f"Batch size {batch_size} isn't divisible by mini batch size {mn_batch_size}"


def generate_agent_name(config: dict) -> str:
    """Function to get agent name to save to/load from file system

    Example names:
    "MLP_B_64-64_P_32-32_V_32-32_relu_2021_01_07__10_32"
    "DRL_LOCAL_PLANNER_2021_01_08__7_14"

    :param config (dict): Dict containing the program arguments
    """
    # if args.custom_mlp:
    #     return (
    #         robot_model
    #         + "_MLP_B_"
    #         + args.body
    #         + "_P_"
    #         + args.pi
    #         + "_V_"
    #         + args.vf
    #         + "_"
    #         + args.act_fn
    #         + "_"
    #         + START_TIME
    #     )
    if config["rl_agent"]["resume"] is None:
        START_TIME = dt.now().strftime("%Y_%m_%d__%H_%M")
        robot_model = rospy.get_param("robot_model")
        architecture_name, encoder_name = config["rl_agent"][
            "architecture_name"
        ], rospy.get_param("space_encoder", "RobotSpecificEncoder")
        agent_name = f"{robot_model}_{architecture_name}_{encoder_name}_{START_TIME}"
        config["agent_name"] = agent_name
        return agent_name


def get_paths(config: dict) -> dict:
    """
    Function to generate agent specific paths

    :param config (dict): Dictionary containing the training configuration
    """
    training_dir = rospkg.RosPack().get_path("training")
    robot_model = rospy.get_param("robot_model")
    simulation_setup = rospkg.RosPack().get_path("arena-simulation-setup")
    agent_name = config["agent_name"]

    PATHS = {
        "model": os.path.join(
            rospkg.RosPack().get_path("rosnav"), "agents", agent_name
        ),
        "tb": os.path.join(training_dir, "training_logs", "tensorboard", agent_name),
        "eval": os.path.join(
            training_dir, "training_logs", "train_eval_log", agent_name
        ),
        "robot_setting": os.path.join(
            simulation_setup,
            "robot",
            robot_model,
            f"{robot_model}.model.yaml",
        ),
        "config": os.path.join(
            rospkg.RosPack().get_path("rosnav"),
            "agents",
            agent_name,
            "training_config.yaml",
        ),
        "curriculum": os.path.join(
            training_dir,
            "configs",
            "training_curriculums",
            config["callbacks"]["training_curriculum"]["training_curriculum_file"],
        ),
    }
    # check for mode
    if config["rl_agent"]["resume"] is None and not config["debug_mode"]:
        os.makedirs(PATHS["model"])
    elif (
        not os.path.isfile(os.path.join(PATHS["model"], f"{agent_name}.zip"))
        and not os.path.isfile(os.path.join(PATHS["model"], "best_model.zip"))
        and not config["debug_mode"]
    ):
        raise FileNotFoundError(
            "Couldn't find model named %s.zip' or 'best_model.zip' in '%s'"
            % (agent_name, PATHS["model"])
        )
    # evaluation log enabled
    if config["monitoring"]["eval_log"] and not config["debug_mode"]:
        if not os.path.exists(PATHS["eval"]):
            os.makedirs(PATHS["eval"])
    else:
        PATHS["eval"] = None
    # tensorboard log enabled
    if config["monitoring"]["tb"] and not config["debug_mode"]:
        if not os.path.exists(PATHS["tb"]):
            os.makedirs(PATHS["tb"])
    else:
        PATHS["tb"] = None

    return PATHS


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


def wait_for_nodes(
    with_ns: bool, n_envs: int, timeout: int = 30, nodes_per_ns: int = 3
) -> None:
    """
    Checks for timeout seconds if all nodes to corresponding namespace are online.

    :param with_ns: (bool) if the system was initialized with namespaces
    :param n_envs: (int) number of virtual environments
    :param timeout: (int) seconds to wait for each ns
    :param nodes_per_ns: (int) usual number of nodes per ns
    """
    if with_ns:
        assert (
            with_ns and n_envs >= 1
        ), f"Illegal number of environments parsed: {n_envs}"
    else:
        assert (
            not with_ns and n_envs == 1
        ), f"Simulation setup isn't compatible with the given number of envs"

    for i in range(n_envs):
        for k in range(timeout):
            ns = f"sim_{str(i + 1)}" if with_ns else ""
            namespaces = rosnode.get_node_names(namespace=ns)

            if len(namespaces) >= nodes_per_ns:
                break

            warnings.warn(
                f"Check if all simulation parts of namespace '{ns}' are running properly"
            )
            warnings.warn("Trying to connect again..")
            assert (
                k < timeout - 1
            ), f"Timeout while trying to connect to nodes of '{ns}'"

            time.sleep(1)


from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.vec_env.base_vec_env import VecEnv


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


import yaml


def load_config(config_name: str) -> dict:
    """
    Load config parameters from config file
    """
    config_location = os.path.join(
        rospkg.RosPack().get_path("training"), "configs", config_name
    )
    with open(config_location, "r", encoding="utf-8") as target:
        config = yaml.load(target, Loader=yaml.FullLoader)

    return config


def load_rew_fnc(config_name: str) -> dict:
    config_location = os.path.join(
        rospkg.RosPack().get_path("training"),
        "configs",
        "reward_functions",
        config_name + ".yaml",
    )
    with open(config_location, "r", encoding="utf-8") as target:
        config = yaml.load(target, Loader=yaml.FullLoader)
    return config


def init_envs(
    config: dict,
    paths: dict,
    ns_for_nodes: bool,
) -> Tuple[VecEnv, VecEnv]:
    import stable_baselines3.common.vec_env as sb3_env

    # instantiate train environment
    # when debug run on one process only
    if not config["debug_mode"] and ns_for_nodes:
        train_env = sb3_env.SubprocVecEnv(
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
        train_env = sb3_env.DummyVecEnv(
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
        eval_env = sb3_env.DummyVecEnv(
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


def init_callbacks(
    config: dict, train_env: VecEnv, eval_env: VecEnv, paths
) -> EvalCallback:
    import tools.staged_train_callback as arena_cb

    # threshold settings for training curriculum
    # type can be either 'succ' or 'rew'
    curriculum_cfg = config["callbacks"]["training_curriculum"]
    stop_train_cfg = config["callbacks"]["stop_training"]
    periodic_eval_cfg = config["callbacks"]["periodic_eval"]

    trainstage_cb = arena_cb.InitiateNewTrainStage(
        n_envs=config["n_envs"],
        treshhold_type=curriculum_cfg["threshold_type"],
        upper_threshold=curriculum_cfg["upper_threshold"],
        lower_threshold=curriculum_cfg["lower_threshold"],
        task_mode=config["task_mode"],
        verbose=1,
    )

    # stop training on reward threshold callback
    stoptraining_cb = StopTrainingOnRewardThreshold(
        treshhold_type=stop_train_cfg["threshold_type"],
        threshold=stop_train_cfg["threshold"],
        verbose=1,
    )

    # evaluation settings
    # n_eval_episodes: number of episodes to evaluate agent on
    # eval_freq: evaluate the agent every eval_freq train timesteps
    eval_cb = EvalCallback(
        eval_env=eval_env,
        train_env=train_env,
        n_eval_episodes=periodic_eval_cfg["n_eval_episodes"],
        eval_freq=periodic_eval_cfg["eval_freq"],
        log_path=paths["eval"],
        best_model_save_path=None if config["debug_mode"] else paths["model"],
        deterministic=True,
        callback_on_eval_end=trainstage_cb,
        callback_on_new_best=stoptraining_cb,
    )

    return eval_cb


def get_ppo_instance(
    config: dict,
    train_env: VecEnv,
    PATHS: dict,
    AgentFactory,
) -> PPO:
    if config["rl_agent"]["architecture_name"] and not config["rl_agent"]["resume"]:
        agent: Union[
            Type[BaseAgent], Type[ActorCriticPolicy]
        ] = AgentFactory.instantiate(config["rl_agent"]["architecture_name"])

        ppo_config = config["rl_agent"]["ppo"]
        ppo_kwargs = {
            "env": train_env,
            "gamma": ppo_config["gamma"],
            "n_steps": ppo_config["n_steps"],
            "ent_coef": ppo_config["ent_coef"],
            "learning_rate": ppo_config["learning_rate"],
            "vf_coef": ppo_config["vf_coef"],
            "max_grad_norm": ppo_config["max_grad_norm"],
            "gae_lambda": ppo_config["gae_lambda"],
            "batch_size": ppo_config["m_batch_size"],
            "n_epochs": ppo_config["n_epochs"],
            "clip_range": ppo_config["clip_range"],
            "tensorboard_log": PATHS["tb"],
            "use_wandb": False
            if config["debug_mode"]
            else config["monitoring"]["use_wandb"],
            "verbose": 1,
        }

        if isinstance(agent, BaseAgent):
            ppo_kwargs["policy"] = agent.type.value
            ppo_kwargs["policy_kwargs"] = agent.get_kwargs()

            model = PPO(**ppo_kwargs)
        elif issubclass(agent, ActorCriticPolicy):
            ppo_kwargs["policy"] = agent

            model = PPO(**ppo_kwargs)
        else:
            arch_name = config["rl_agent"]["architecture_name"]
            raise TypeError(
                f"Registered agent class {arch_name} is neither of type"
                "'BaseAgent' or 'ActorCriticPolicy'!"
            )
    else:
        agent_name = config["agent_name"]
        # load flag
        if os.path.isfile(os.path.join(PATHS["model"], f"{agent_name}.zip")):
            model = PPO.load(os.path.join(PATHS["model"], agent_name), train_env)
        elif os.path.isfile(os.path.join(PATHS["model"], "best_model.zip")):
            model = PPO.load(os.path.join(PATHS["model"], "best_model"), train_env)
        else:
            raise FileNotFoundError(
                f"Could not find model file for agent {agent_name}!"
                "You might need to change the 'agent_name' in the config file "
                "according to the name of the parent directory of the desired model."
            )
        update_hyperparam_model(model, PATHS, config)
        if not config["debug_mode"] and config["monitoring"]["use_wandb"]:
            init_wandb(model)
    return model


def populate_ros_params(params):
    rospy.set_param("task_mode", params["task_mode"])
    rospy.set_param(
        "is_action_space_discrete", params["rl_agent"]["discrete_action_space"]
    )
    rospy.set_param("goal_radius", params["goal_radius"])


def populate_ros_configs(config):
    rospy.set_param("debug_mode", config["debug_mode"])
