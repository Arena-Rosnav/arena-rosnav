#!/usr/bin/env python
import sys
import time

import rospy
from rosnav.model.agent_factory import AgentFactory
from rosnav.model.base_agent import BaseAgent
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize
from std_msgs.msg import Empty
from tools.argsparser import parse_training_args
from tools.env_utils import init_envs
from tools.general import *
from tools.model_utils import get_ppo_instance, init_callbacks
from tools.ros_param_distributor import *


def on_shutdown(model):
    model.env.close()
    sys.exit()


def main():
    args, _ = parse_training_args()

    config = load_config(args.config)

    populate_ros_configs(config)

    # in debug mode, we emulate multiprocessing on only one process
    # in order to be better able to locate bugs
    if config["debug_mode"]:
        rospy.init_node("debug_node", disable_signals=False)

    # generate agent name and model specific paths
    generate_agent_name(config)
    paths = get_paths(config)

    print("________ STARTING TRAINING WITH:  %s ________\n" % config["agent_name"])

    # for training with start_arena_flatland.launch
    ns_for_nodes = rospy.get_param("/ns_for_nodes", True)

    # check if simulations are booted
    wait_for_nodes(with_ns=ns_for_nodes, n_envs=config["n_envs"], timeout=5)

    # initialize hyperparameters (save to/ load from json)
    config = initialize_config(
        paths=paths,
        config=config,
        n_envs=config["n_envs"],
        debug_mode=config["debug_mode"],
    )

    populate_ros_params(config)

    agent_description: BaseAgent = AgentFactory.instantiate(
        config["rl_agent"]["architecture_name"]
    )

    train_env, eval_env = init_envs(agent_description, config, paths, ns_for_nodes)
    eval_cb = init_callbacks(config, train_env, eval_env, paths)
    model = get_ppo_instance(agent_description, config, train_env, paths)

    rospy.on_shutdown(lambda: on_shutdown(model))

    ## Save model once
    if not config["debug_mode"]:
        model.save(os.path.join(paths["model"], "best_model"))
        if isinstance(train_env, VecNormalize):
            train_env.save(os.path.join(paths["model"], "vec_normalize.pkl"))

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

    model.env.close()

    sys.exit()


if __name__ == "__main__":
    main()
