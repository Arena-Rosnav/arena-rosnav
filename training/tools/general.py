import os
import random
import string
import time
import warnings
from typing import Tuple

import numpy as np
import rosnode
import rospy
import yaml

from .model_utils import check_batch_size
from .constants import TRAINING_CONSTANTS


def initialize_config(
    paths: dict,
    config: dict,
    n_envs: int = 1,
    debug_mode: bool = False,
) -> dict:
    """
    Initialize config file for training and save it to agent directory

    :param PATHS: dictionary containing model specific paths
    :param config: dictionary containing training configurations
    :param n_envs: number of envs
    """
    import rosnav.model.custom_policy
    import rosnav.model.custom_sb3_policy

    config["robot"] = rospy.get_param("model")
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
        write_config_yaml(config, paths)
    print_hyperparameters(config["rl_agent"]["ppo"])

    return config


def write_config_yaml(config: dict, paths: dict) -> None:
    """
    Write training_config.yaml to agent directory

    :param hyperparams: dict containing model specific hyperparameters
    :param paths: dictionary containing model specific paths
    """
    with open(paths["config"], "w") as outfile:
        yaml.dump(config, outfile, default_flow_style=False)


def print_hyperparameters(hyperparams: dict) -> None:
    print("\n--------------------------------")
    print("         HYPERPARAMETERS         \n")
    for param, param_val in hyperparams.items():
        print("{:30s}{:<10s}".format(f"{param}:", str(param_val)))
    print("--------------------------------\n\n")


def get_paths(config: dict) -> dict:
    """
    Function to generate agent specific paths

    :param config (dict): Dictionary containing the training configuration
    """
    agent_name = config["agent_name"]

    BASE_PATHS = TRAINING_CONSTANTS.PATHS
    PATHS = {
        "model": BASE_PATHS.MODEL(agent_name),
        "tb": BASE_PATHS.TENSORBOARD(agent_name),
        "eval": BASE_PATHS.EVAL(agent_name),
        "robot_setting": BASE_PATHS.ROBOT_SETTING(rospy.get_param("robot_model")),
        "config": BASE_PATHS.AGENT_CONFIG(agent_name),
        "curriculum": BASE_PATHS.CURRICULUM(
            config["callbacks"]["training_curriculum"]["training_curriculum_file"]
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
            f"""Couldn't find model named {agent_name}.zip' or 'best_model.zip' in '{PATHS["model"]}'"""
        )
    # evaluation log enabled
    if config["monitoring"]["eval_log"] and not config["debug_mode"]:
        if not os.path.exists(PATHS["eval"]):
            os.makedirs(PATHS["eval"])
    else:
        PATHS["eval"] = None
    # tensorboard log enabled
    if config["monitoring"]["use_wandb"] and not config["debug_mode"]:
        if not os.path.exists(PATHS["tb"]):
            os.makedirs(PATHS["tb"])
    else:
        PATHS["tb"] = None

    return PATHS


def wait_for_nodes(
    with_ns: bool, n_envs: int, timeout: int = 30, nodes_per_ns: int = 2
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
        ), "Simulation setup isn't compatible with the given number of envs"

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


def load_config(config_name: str) -> dict:
    """
    Load config parameters from config file
    """

    config_location = TRAINING_CONSTANTS.PATHS.TRAINING_CONFIGS(config_name)
    with open(config_location, "r", encoding="utf-8") as target:
        config = yaml.load(target, Loader=yaml.FullLoader)

    return config


def generate_agent_name(config: dict) -> str:
    """Function to get agent name to save to/load from file system

    Example names:
    "MLP_B_64-64_P_32-32_V_32-32_relu_2021_01_07__10_32"
    "DRL_LOCAL_PLANNER_2021_01_08__7_14"

    :param config (dict): Dict containing the program arguments
    """
    if config["rl_agent"]["resume"] is None:
        agent_name = TRAINING_CONSTANTS.generate_agent_name(
            architecture_name=config["rl_agent"]["architecture_name"]
        )
        config["agent_name"] = agent_name
        return agent_name
    else:
        config["agent_name"] = config["rl_agent"]["resume"]
        return config["rl_agent"]["resume"]


def generate_discrete_action_dict(
    linear_range: Tuple[float, float],
    angular_range: Tuple[float, float],
    num_linear_actions: int,
    num_angular_actions: int,
):
    NAME_LEN = 12  # len for random action name

    linear_actions = np.linspace(
        linear_range[0], linear_range[1], num_linear_actions, dtype=np.float16
    )
    angular_actions = np.linspace(
        angular_range[0], angular_range[1], num_angular_actions, dtype=np.float16
    )

    discrete_action_space = [
        (float(linear_action), float(angular_action))
        for linear_action in linear_actions
        for angular_action in angular_actions
    ]
    if (0, 0) not in discrete_action_space:
        discrete_action_space.append((0, 0))

    return [
        {
            "name": "".join(random.sample(string.ascii_lowercase, NAME_LEN)),
            "linear": linear,
            "angular": angular,
        }
        for linear, angular in discrete_action_space
    ]
