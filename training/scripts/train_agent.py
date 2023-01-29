#!/usr/bin/env python
<<<<<<< HEAD
import sys, rospy, time
=======
from typing import Type, Union

import os, sys, rospy, time

from std_msgs.msg import Empty

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.callbacks import (
    EvalCallback,
    StopTrainingOnRewardThreshold,
)
from stable_baselines3.common.policies import ActorCriticPolicy, BasePolicy
>>>>>>> dev

from rosnav.model.agent_factory import AgentFactory
from tools.argsparser import parse_training_args
from tools.general import *
from tools.model_utils import init_callbacks, get_ppo_instance
from tools.env_utils import init_envs


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
    PATHS = get_paths(config)

    print("________ STARTING TRAINING WITH:  %s ________\n" % config["agent_name"])

    # for training with start_arena_flatland.launch
    ns_for_nodes = rospy.get_param("/ns_for_nodes", True)

    # check if simulations are booted
    wait_for_nodes(with_ns=ns_for_nodes, n_envs=config["n_envs"], timeout=5)

    # initialize hyperparameters (save to/ load from json)
    config = initialize_config(
        PATHS=PATHS,
        config=config,
        n_envs=config["n_envs"],
        debug_mode=config["debug_mode"],
    )

    populate_ros_params(config)

    train_env, eval_env = init_envs(config, PATHS, ns_for_nodes)
    eval_cb = init_callbacks(config, train_env, eval_env, PATHS)
    model = get_ppo_instance(config, train_env, PATHS, AgentFactory)

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

<<<<<<< HEAD
    rospy.on_shutdown(model.env.close())
    print(f"Time passed: {time.time()-start}s. \n Training script will be terminated..")
=======
    model.learn(
        total_timesteps=n_timesteps,
        callback=eval_cb,
        reset_num_timesteps=True,
    )

    # update the timesteps the model has trained in total
    # update_total_timesteps_json(n_timesteps, PATHS)

    model.env.close()
    print(f"Time passed: {time.time()-start}s")
    print("Training script will be terminated")

    publisher = rospy.Publisher("training_finished", Empty, queue_size=10)

    for _ in range(10):
        publisher.publish(Empty())
        rospy.sleep(0.1)

>>>>>>> dev
    sys.exit()


if __name__ == "__main__":
    main()
