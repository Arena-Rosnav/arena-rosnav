#!/usr/bin/env python
import sys
import time

import rospy
from rosnav.model.agent_factory import AgentFactory
from rosnav.model.base_agent import BaseAgent
from tools.argsparser import parse_training_args
from tools.env_utils import make_envs
from tools.general import generate_agent_name, get_paths, initialize_config, load_config
from tools.model_utils import (
    get_ppo_instance,
    init_callbacks,
    save_model,
    load_policies,
)
from tools.ros_param_distributor import populate_ros_params


def on_shutdown(model, paths: dict):
    model.env.close()
    if not rospy.get_param("debug_mode", False):
        save_model(model, paths, "last_model")
    sys.exit()


def main():
    args, _ = parse_training_args()

    if (config_name := args.config) == "":
        raise RuntimeError("No config specified. Please specify a config file.")

    config = load_config(config_name)
    rospy.set_param("debug_mode", config["debug_mode"])

    # in debug mode, we emulate multiprocessing on only one process
    # in order to be better able to locate bugs
    if config["debug_mode"]:
        rospy.init_node("debug_node", disable_signals=False)

    # generate agent name and model specific paths
    generate_agent_name(config)
    paths = get_paths(config)

    print("________ STARTING TRAINING WITH:  %s ________\n" % config["agent_name"])

    # initialize hyperparameters (save to/ load from json)
    config = initialize_config(paths=paths, config=config)

    populate_ros_params(config, paths)

    load_policies()
    agent_description: BaseAgent = AgentFactory.instantiate(
        config["rl_agent"]["architecture_name"]
    )

    train_env, eval_env, observation_space_manager = make_envs(
        agent_description, config, paths
    )
    eval_cb = init_callbacks(config, train_env, eval_env, paths)
    model = get_ppo_instance(
        agent_description, observation_space_manager, config, train_env, paths
    )

    # Get the number of parameters
    # num_params = sum(p.numel() for p in model.policy.parameters())

    rospy.on_shutdown(lambda: on_shutdown(model, paths))

    # start training
    start = time.time()
    try:
        model.learn(
            total_timesteps=config["n_timesteps"] or 40000000,
            callback=eval_cb,
            reset_num_timesteps=True,
        )
    except KeyboardInterrupt:
        print("KeyboardInterrupt..")

    print(f"Time passed: {time.time()-start}s. \n Training script will be terminated..")

    sys.exit()


if __name__ == "__main__":
    main()
