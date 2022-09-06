#!/usr/bin/env python
import sys
import time

import rospy
from rosnav.model.agent_factory import AgentFactory
from tools.argsparser import parse_training_args
from tools.custom_mlp_utils import *
from tools.train_agent_utils import *


def main(args):
    config = load_config(args.config)
    rospy.set_param("debug_mode", config["debug_mode"])

    # in debug mode, we emulate multiprocessing on only one process
    # in order to be better able to locate bugs
    # if args.debug:
    if config["debug_mode"]:
        rospy.init_node("debug_node", disable_signals=False)

    # generate agent name and model specific paths
    AGENT_NAME, PATHS = get_agent_name_and_paths(config)

    print("________ STARTING TRAINING WITH:  %s ________\n" % AGENT_NAME)

    # for training with start_arena_flatland.launch
    ns_for_nodes = not rospy.get_param("single_env", True)

    # check if simulations are booted
    wait_for_nodes(with_ns=ns_for_nodes, n_envs=config["n_envs"], timeout=5)

    # initialize hyperparameters (save to/ load from json)
    params = init_hyperparameters(
        PATHS=PATHS,
        load_target=config["resume"],
        config_name=config["hyperparameter_file"],
        n_envs=config["n_envs"],
        debug_mode=config["debug_mode"],
    )

    rospy.set_param("task_mode", params["task_mode"])
    rospy.set_param("is_action_space_discrete", params["discrete_action_space"])

    train_env, eval_env = init_envs(
        config,
        params,
        PATHS,
        ns_for_nodes,
        load_rew_fnc(config["reward_function_file"]),
    )
    eval_cb = init_callbacks(config, params, train_env, eval_env, PATHS)
    model = get_ppo_instance(config, params, train_env, PATHS, AGENT_NAME, AgentFactory)

    # set num of timesteps to be generated
    n_timesteps = config["n_timesteps"] or 40000000

    # start training
    start = time.time()
    try:
        model.learn(
            total_timesteps=n_timesteps,
            callback=eval_cb,
            reset_num_timesteps=True,
        )
    except KeyboardInterrupt:
        print("KeyboardInterrupt..")
    # finally:
    # update the timesteps the model has trained in total
    # update_total_timesteps_json(n_timesteps, PATHS)

    model.env.close()
    print(
        f"Time passed: {time.time()-start}s. \n" "Training script will be terminated.."
    )
    sys.exit()


if __name__ == "__main__":
    args, _ = parse_training_args()
    main(args)
