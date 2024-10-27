#!/usr/bin/env python
import sys
import time
from typing import TypedDict

import rl_utils.cfg as cfg
import rosnav_rl.cfg as rl_cfg
import rosnav_rl.rl_agent as Rosnav_RL
import rospy
from rl_utils.state_container import SimulationStateContainer
from rl_utils.utils.type_alias.observation import PathsDict
from tools.argsparser import parse_training_args
from tools.env_utils import make_envs
from tools.general import (
    create_directories,
    get_paths,
    load_config,
    print_base_model,
    save_model_and_exit,
    write_config_yaml,
)
from tools.model_utils import init_sb3_callbacks, setup_wandb
from tools.states import get_arena_states
from tools.config import unpack_framework_config, load_training_config


class SB3PPOTrainMethodArguments(TypedDict):
    total_timesteps: int
    callback: callable
    progress_bar: bool = False
    log_interval: int = 1
    tb_log_name: str = "PPO"
    reset_num_timesteps: bool = False


def initialize_ros_debug_node(debug_mode: bool) -> None:
    rospy.set_param("debug_mode", debug_mode)
    if debug_mode:
        rospy.init_node("debug_node", disable_signals=False)


def setup_paths_and_directories(
    agent_cfg: cfg.AgentCfg,
    callbacks_cfg: cfg.CallbacksCfg,
    monitoring_cfg: cfg.MonitoringCfg,
    general_cfg: cfg.GeneralCfg,
) -> PathsDict:
    paths = get_paths(
        agent_name=agent_cfg.name,
        curriculum_file=callbacks_cfg.training_curriculum.curriculum_file,
    )
    if not general_cfg.debug_mode:
        create_directories(
            paths=paths,
            resume_name=agent_cfg.framework.model.resume,
            log_evaluation=monitoring_cfg.eval_metrics,
            use_wandb=monitoring_cfg.wandb,
        )
    return paths


def main(config_name: str = "") -> None:
    config: cfg.TrainingCfg = load_training_config(config_name)
    (
        general_cfg,
        callbacks_cfg,
        monitoring_cfg,
        task_cfg,
        robot_cfg,
        normalization_cfg,
        profiling_cfg,
    ) = unpack_framework_config(config.framework_cfg)
    agent_cfg: rl_cfg.AgentCfg = config.agent_cfg

    initialize_ros_debug_node(general_cfg.debug_mode)
    paths = setup_paths_and_directories(
        agent_cfg, callbacks_cfg, monitoring_cfg, general_cfg
    )

    sim_state_container: SimulationStateContainer = get_arena_states(
        goal_radius=general_cfg.goal_radius,
        max_steps=general_cfg.max_num_moves_per_eps,
        is_discrete=agent_cfg.action_space.is_discrete,
        safety_distance=general_cfg.safety_distance,
        robot_cfg=robot_cfg,
        task_modules_cfg=task_cfg,
    )

    agent_instance = Rosnav_RL.RL_Agent(
        agent_cfg=agent_cfg,
        simulation_state_container=sim_state_container,
    )

    train_env, eval_env = make_envs(
        rl_agent=agent_instance,
        simulation_state_container=sim_state_container,
        agent_cfg=agent_cfg,
        general_cfg=general_cfg,
        callback_cfg=callbacks_cfg,
        normalization_cfg=normalization_cfg,
        monitoring_cfg=monitoring_cfg,
        profiling_cfg=profiling_cfg,
    )

    eval_cb = init_sb3_callbacks(
        train_env=train_env,
        eval_env=eval_env,
        n_envs=general_cfg.n_envs,
        tm_modules=task_cfg.tm_modules,
        callback_cfg=callbacks_cfg,
        model_save_path=paths["model"],
        eval_log_path=paths["eval"],
        debug_mode=general_cfg.debug_mode,
    )

    agent_instance.model.initialize(
        env=train_env,
        no_gpu=general_cfg.no_gpu,
        tensorboard_log_path=paths["tb"] if not general_cfg.debug_mode else None,
    )

    if not general_cfg.debug_mode:
        write_config_yaml(config.model_dump(), paths["config"])
        if monitoring_cfg.wandb:
            setup_wandb(train_cfg=config, rl_model=agent_instance.model)

    print(f"________ STARTING TRAINING WITH:  {agent_cfg.name} ________\n")
    print_base_model(config)

    rospy.on_shutdown(
        lambda: save_model_and_exit(
            rl_model=agent_instance,
            dirpath=paths["model"],
            checkpoint_name="last_model",
        )
    )

    start = time.time()
    agent_instance.model.train(
        **SB3PPOTrainMethodArguments(
            total_timesteps=general_cfg.n_timesteps,
            callback=eval_cb,
            progress_bar=general_cfg.show_progress_bar,
        )
    )
    print(f"Time passed: {time.time()-start}s. \n Training script will be terminated..")
    sys.exit()


if __name__ == "__main__":
    args, _ = parse_training_args()

    if (config_name := args.config) == "":
        raise RuntimeError("No config specified. Please specify a config file.")

    main(config_name=config_name)
