#!/usr/bin/env python
import sys
import time

import rospy
from rosnav.model.agent_factory import AgentFactory
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize
from std_msgs.msg import Empty
from tools.argsparser import parse_training_args
from tools.env_utils import init_envs
from tools.general import *
from tools.model_utils import get_ppo_instance, init_callbacks
from tools.ros_param_distributor import *

"""
1. X Deploy LSTM and FrameStacking Agents
2. Barn Maps testing
3. Metrics for LSTM and FrameStacking Agents
4. X Collision detection form behind
5. X More Maps on Barn like Specs (smaller res, smaller maps)
6. Reward Shaping
7. LSTM testing
8. X Reduce observation space
9. X Discrete Action Space
10. X Update Observation Space on loading RPPO
11. ---> Log training episode metrics
12. See approaches of other teams
13. ---> Check if Env are paused when Map Generator is running a new map
"""
"""
TODO:
- X deploy reduced laser encoder
- X test full map
- X finish: Discrete Action Space (also deployment)
- Check saving VecNorm on StackedObs
6. Reward Shaping
"""


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

    rospy.on_shutdown(lambda: on_shutdown(model))

    ## Save model once
    if not config["debug_mode"]:
        model.save(os.path.join(PATHS["model"], "best_model"))
        if isinstance(train_env, VecNormalize):
            train_env.save(os.path.join(PATHS["model"], "vec_normalize.pkl"))

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

    # Send Task finished to Backend
    if rospy.get_param("/is_webapp_docker", False):
        publisher = rospy.Publisher("training_finished", Empty, queue_size=10)

        while publisher.get_num_connections() <= 0:
            pass

        publisher.publish(Empty())

    sys.exit()


if __name__ == "__main__":
    main()
