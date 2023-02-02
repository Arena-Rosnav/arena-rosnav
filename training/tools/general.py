import os, rospy
import rosnode
import rospkg
import time
import warnings
import yaml

from datetime import datetime as dt

from stable_baselines3 import PPO

from .model_utils import check_batch_size


def populate_ros_params(params):
    rospy.set_param("task_mode", params["task_mode"])
    rospy.set_param(
        "is_action_space_discrete", params["rl_agent"]["discrete_action_space"]
    )
    rospy.set_param("goal_radius", params["goal_radius"])


def populate_ros_configs(config):
    rospy.set_param("debug_mode", config["debug_mode"])


def initialize_config(
    PATHS: dict,
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
        f"{config_name}.yaml",
    )
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
        START_TIME = dt.now().strftime("%Y_%m_%d__%H_%M_%S")
        robot_model = rospy.get_param("robot_model")
        architecture_name, encoder_name = config["rl_agent"][
            "architecture_name"
        ], rospy.get_param("space_encoder", "RobotSpecificEncoder")
        agent_name = f"{robot_model}_{architecture_name}_{encoder_name}_{START_TIME}"
        config["agent_name"] = agent_name
        return agent_name
