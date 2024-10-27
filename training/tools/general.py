import json
import os
import random
import string
import sys
import time
import warnings
from typing import Tuple

import numpy as np
import rosnode
import rospy
import yaml
from pydantic import BaseModel
from pygments import highlight
from pygments.formatters import TerminalFormatter
from pygments.lexers import get_lexer_by_name
from rosnav_rl.model import StableBaselinesAgent

from .constants import TRAINING_CONSTANTS


def write_config_yaml(config: dict, path: str) -> None:
    with open(path, "w") as outfile:
        yaml.dump(config, outfile, default_flow_style=False)


def print_dict(hyperparams: dict) -> None:
    print("\n--------------------------------")
    print("         HYPERPARAMETERS         \n")
    for param, param_val in hyperparams.items():
        print("{:30s}{:<10s}".format(f"{param}:", str(param_val)))
    print("--------------------------------\n\n")


def print_base_model(hyperparams: BaseModel) -> None:
    print("\n--------------------------------")
    print("         HYPERPARAMETERS         \n")
    yaml_str = yaml.dump(
        hyperparams.model_dump(), default_flow_style=False, sort_keys=False
    )
    colorful_yaml = highlight(yaml_str, get_lexer_by_name("yaml"), TerminalFormatter())
    print(colorful_yaml)
    print("--------------------------------\n\n")


def get_paths(
    agent_name: str,
    curriculum_file: str,
    # resume_name: str,
    # checkpoint_name: str,
    # debug_mode: bool,
    # use_wandb: bool = False,
    # log_evaluation: bool = False,
) -> dict:
    """
    Function to generate agent specific paths

    :param config (dict): Dictionary containing the training configuration
    """
    BASE_PATHS = TRAINING_CONSTANTS.PATHS
    PATHS = {
        "model": BASE_PATHS.MODEL(agent_name),
        "tb": BASE_PATHS.TENSORBOARD(agent_name),
        "eval": BASE_PATHS.EVAL(agent_name),
        "robot_setting": BASE_PATHS.ROBOT_SETTING(rospy.get_param("robot_model")),
        "config": BASE_PATHS.AGENT_CONFIG(agent_name),
        "curriculum": BASE_PATHS.CURRICULUM(curriculum_file),
    }

    return PATHS


def create_directories(
    paths: dict,
    resume_name: str,
    log_evaluation: bool,
    use_wandb: bool,
) -> None:
    create_model_directory(paths, resume_name)
    paths["eval"] = create_evaluation_directory(paths, log_evaluation)
    paths["tb"] = create_tensorboard_directory(paths, use_wandb)


def create_model_directory(paths: dict, resume_name: str) -> None:
    """
    Create model directory if not in debug mode and resume_name is None.
    Raise FileNotFoundError if checkpoint is not found.

    :param PATHS: Dictionary containing paths
    :param resume_name: Name of the resume file
    :param checkpoint_name: Name of the checkpoint file
    :param debug_mode: Boolean indicating if debug mode is enabled
    """
    if resume_name is None:
        os.makedirs(paths["model"])


def create_evaluation_directory(paths: dict, log_evaluation: bool) -> str:
    """
    Create evaluation directory if log_evaluation is enabled and not in debug mode.

    :param PATHS: Dictionary containing paths
    :param log_evaluation: Boolean indicating if evaluation logging is enabled
    :return: Path to evaluation directory or None
    """
    if log_evaluation:
        if not os.path.exists(paths["eval"]):
            os.makedirs(paths["eval"])
        return paths["eval"]
    return None


def create_tensorboard_directory(paths: dict, use_wandb: bool) -> str:
    """
    Create tensorboard directory if use_wandb is enabled and not in debug mode.

    :param PATHS: Dictionary containing paths
    :param use_wandb: Boolean indicating if wandb is used
    :param debug_mode: Boolean indicating if debug mode is enabled
    :return: Path to tensorboard directory or None
    """
    if use_wandb:
        if not os.path.exists(paths["tb"]):
            os.makedirs(paths["tb"])
        return paths["tb"]
    return None


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
    if with_ns and n_envs < 1:
        raise ValueError(f"Illegal number of environments parsed: {n_envs}")
    elif not with_ns and n_envs != 1:
        raise ValueError(
            "Simulation setup isn't compatible with the given number of envs"
        )

    for i in range(n_envs):
        ns = f"sim_{str(i + 1)}" if with_ns else ""
        for k in range(timeout):
            namespaces = rosnode.get_node_names(namespace=ns)

            if len(namespaces) >= nodes_per_ns:
                break

            warnings.warn(
                f"Check if all simulation parts of namespace '{ns}' are running properly"
            )
            warnings.warn("Trying to connect again..")
            if k >= timeout - 1:
                raise TimeoutError(
                    f"Timeout while trying to connect to nodes of '{ns}'"
                )

            time.sleep(1)


def load_config(file_path: str) -> dict:
    """
    Load config parameters from config file
    """
    with open(file_path, "r", encoding="utf-8") as target:
        config = yaml.load(target, Loader=yaml.FullLoader)
    return config


def save_model_and_exit(
    rl_model: StableBaselinesAgent, dirpath: str, checkpoint_name: str
):
    if not rospy.get_param("debug_mode", False):
        rl_model.save(dirpath=dirpath, checkpoint_name=checkpoint_name)
    rl_model.model.env.close()
    sys.exit()
