import argparse
import os
import pathlib

import rospkg
import json
import yaml


DEFAULT_CONFIG = "training_config.yaml"
CONFIG_LOCATION = os.path.join(
    rospkg.RosPack().get_path("training"), "configs", DEFAULT_CONFIG
)


def get_parsed_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--path",
        "-p",
        type=str,
        metavar="[path to the json hyperparam file]",
        help="Path to the json hyperparam file which will "
        "get translated to the new yaml format.",
    )
    return parser.parse_args()


def load_files(json_path):
    with open(json_path, "r") as file:
        hyperparams = json.load(file)
    with open(CONFIG_LOCATION, "r", encoding="utf-8") as target:
        config = yaml.load(target, Loader=yaml.FullLoader)
    return hyperparams, config


def transfer_entries(hyperparams, config):
    # general config
    config["agent_name"] = hyperparams["agent_name"]
    config["robot"] = hyperparams["robot"]
    config["goal_radius"] = hyperparams["goal_radius"]
    config["task_mode"] = hyperparams["task_mode"]
    config["max_num_moves_per_eps"] = hyperparams["train_max_steps_per_episode"]

    # callbacks config
    config["callbacks"]["periodic_eval"]["max_steps_per_episode"] = hyperparams[
        "eval_max_steps_per_episode"
    ]
    config["callbacks"]["training_curriculum"]["curr_stage"] = hyperparams["curr_stage"]

    # agent specific config
    config["rl_agent"]["reward_fnc"] = hyperparams["reward_fnc"]
    config["rl_agent"]["discrete_action_space"] = hyperparams["discrete_action_space"]
    config["rl_agent"]["normalize"] = hyperparams["normalize"]

    # ppo config
    config["rl_agent"]["ppo"]["batch_size"] = hyperparams["batch_size"]
    config["rl_agent"]["ppo"]["gammma"] = hyperparams["gamma"]
    config["rl_agent"]["ppo"]["n_steps"] = hyperparams["n_steps"]
    config["rl_agent"]["ppo"]["ent_coef"] = hyperparams["ent_coef"]
    config["rl_agent"]["ppo"]["learning_rate"] = hyperparams["learning_rate"]
    config["rl_agent"]["ppo"]["vf_coef"] = hyperparams["vf_coef"]
    config["rl_agent"]["ppo"]["max_grad_norm"] = hyperparams["max_grad_norm"]
    config["rl_agent"]["ppo"]["gae_lambda"] = hyperparams["gae_lambda"]
    config["rl_agent"]["ppo"]["m_batch_size"] = hyperparams["m_batch_size"]
    config["rl_agent"]["ppo"]["n_epochs"] = hyperparams["n_epochs"]
    config["rl_agent"]["ppo"]["clip_range"] = hyperparams["clip_range"]

    return config


def extract_yaml_file_path(path_to_json):
    path_to_json = pathlib.Path(path_to_json)
    return path_to_json.parent / "training_config.yaml"


def main():
    parsed_args = get_parsed_args()

    hyperparams, config = load_files(parsed_args.path)

    config = transfer_entries(hyperparams, config)

    with open(extract_yaml_file_path(parsed_args.path), "w") as outfile:
        yaml.dump(config, outfile, default_flow_style=False)


if __name__ == "__main__":
    main()
