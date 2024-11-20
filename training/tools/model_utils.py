import os
import sys
from typing import Callable

import rospy
import wandb
from rl_utils.utils.eval_callbacks.staged_train_callback import InitiateNewTrainStage
from rl_utils.utils.learning_rate_schedules.linear import linear_decay
from rosnav.model.base_agent import BaseAgent
from sb3_contrib import RecurrentPPO
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    EvalCallback,
    StopTrainingOnRewardThreshold,
)
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.utils import configure_logger
from stable_baselines3.common.vec_env.base_vec_env import VecEnv
from stable_baselines3.common.vec_env.vec_normalize import VecNormalize


def setup_wandb(config: dict, agent: PPO) -> None:
    wandb.login(key="58b5a2040f5cc9d5c3a7d6102877515716298192")
    wandb.init(
        project="Arena-RL",
        entity=None,
        sync_tensorboard=True,
        monitor_gym=True,
        save_code=True,
        config=config,
    )
    wandb.watch(agent.policy)


def check_batch_size(n_envs: int, batch_size: int, mn_batch_size: int) -> None:
    assert (
        batch_size >= mn_batch_size
    ), f"Mini batch size {mn_batch_size} is bigger than batch size {batch_size}"

    assert (
        batch_size % mn_batch_size == 0
    ), f"Batch size {batch_size} isn't divisible by mini batch size {mn_batch_size}"

    assert (
        batch_size % n_envs == 0
    ), f"Batch size {batch_size} isn't divisible by n_envs {n_envs}"

    assert (
        batch_size % mn_batch_size == 0
    ), f"Batch size {batch_size} isn't divisible by mini batch size {mn_batch_size}"


def update_hyperparam_model(model: PPO, PATHS: dict, config: dict) -> None:
    """
    Updates parameter of loaded PPO agent when it was manually changed in the configs yaml.

    :param model(object, PPO): loaded PPO agent
    :param PATHS: program relevant paths
    :param params: dictionary containing loaded hyperparams
    :param n_envs: number of parallel environments
    """

    def update(model, key, new_val):
        if getattr(model, key) != new_val:
            print(
                "{:40}{:<10s}".format(
                    f"Updating '{key}':",
                    f"{str(getattr(model, key))}->{str(new_val)}",
                )
            )
            setattr(model, key, new_val)

    print("\n--------------------------------")
    print("UPDATING MODEL HYPERPARAMETER...")
    print("(no change -> no print below)")

    ppo_params = config["rl_agent"]["ppo"]
    update(model, "batch_size", ppo_params["m_batch_size"])
    update(model, "gamma", ppo_params["gamma"])
    update(model, "n_steps", ppo_params["n_steps"])
    update(model, "ent_coef", ppo_params["ent_coef"])
    update(model, "vf_coef", ppo_params["vf_coef"])
    update(model, "max_grad_norm", ppo_params["max_grad_norm"])
    update(model, "gae_lambda", ppo_params["gae_lambda"])
    update(model, "n_epochs", ppo_params["n_epochs"])

    if not config["rl_agent"]["lr_schedule"]["enabled"]:
        update(model, "learning_rate", ppo_params["learning_rate"])
    else:
        setattr(model, "learning_rate", load_lr_schedule(config))
    model._setup_lr_schedule()

    if model.n_envs != config["n_envs"]:
        print(
            "{:40}{:<10s}".format(
                "Updating 'n_envs':",
                f"{str(model.n_envs)}->" + str(config["n_envs"]),
            )
        )
        model.update_n_envs()
        model.n_envs = config["n_envs"]
        model.rollout_buffer.buffer_size = ppo_params["n_steps"]
    if not config["debug_mode"] and config["monitoring"]["use_wandb"]:
        model.tensorboard_log = PATHS["tb"]
        logger = configure_logger(1, PATHS["tb"], "run", False)
        model._logger = logger
    if config["debug_mode"]:
        model.tensorboard_log = None
        model._logger = None

    model._setup_rollout_buffer()

    print("--------------------------------\n")


def load_lr_schedule(config: dict) -> Callable:
    lr_schedule_cfg = config["rl_agent"]["lr_schedule"]
    lr_schedule = None
    if lr_schedule_cfg["type"] == "linear":
        lr_schedule = linear_decay(**lr_schedule_cfg["settings"])
    else:
        raise NotImplementedError(
            f"Learning rate schedule '{lr_schedule_cfg['type']}' not implemented!"
        )
    return lr_schedule


def save_model(model: PPO, paths: dict, file_name: str = "best_model") -> None:
    print("Saving model to: ", paths["model"])
    model.save(os.path.join(paths["model"], file_name))

    if isinstance(model.env, VecNormalize):
        model.env.save(os.path.join(paths["model"], f"vec_normalize_{file_name}.pkl"))
    elif isinstance(model.env.venv, VecNormalize):
        model.env.venv.save(
            os.path.join(paths["model"], f"vec_normalize_{file_name}.pkl")
        )


def get_ppo_instance(
    agent_description: BaseAgent,
    observation_manager,
    config: dict,
    train_env: VecEnv,
    paths: dict,
) -> PPO:
    new_model: bool = (
        config["rl_agent"]["architecture_name"] and not config["rl_agent"]["resume"]
    )
    if new_model:
        model = instantiate_new_model(
            agent_description, observation_manager, config, train_env, paths
        )
    else:
        model = load_model(config, train_env, paths)
        update_hyperparam_model(model, paths, config)

    wandb_logging: bool = not config["debug_mode"] and config["monitoring"]["use_wandb"]

    if wandb_logging:
        setup_wandb(config, model)

    ## Save model once

    if not rospy.get_param("debug_mode") and new_model:
        save_model(model, paths)
    return model


def instantiate_new_model(
    agent_description: BaseAgent,
    observation_manager,
    config: dict,
    train_env: VecEnv,
    PATHS: dict,
) -> PPO:
    ppo_config = config["rl_agent"]["ppo"].copy()
    ppo_config["batch_size"] = ppo_config["m_batch_size"]
    del ppo_config["m_batch_size"]

    # potentially get learning rate schedule
    learning_rate = (
        load_lr_schedule(config)
        if config["rl_agent"]["lr_schedule"]["enabled"]
        else ppo_config["learning_rate"]
    )
    ppo_config["learning_rate"] = learning_rate

    ppo_kwargs = {
        "env": train_env,
        **ppo_config,
        "tensorboard_log": PATHS["tb"],
        # "use_wandb": False if config["debug_mode"] else config["monitoring"]["use_wandb"],
        "verbose": config["monitoring"]["cmd_line_logging"]["training_metrics"][
            "enabled"
        ],
        "device": "cpu" if config["no_gpu"] else "auto",
    }

    if isinstance(agent_description, BaseAgent):
        ppo_kwargs["policy"] = agent_description.type.value
        policy_kwargs = agent_description.get_kwargs()
        policy_kwargs["features_extractor_kwargs"][
            "observation_space_manager"
        ] = observation_manager
        policy_kwargs["features_extractor_kwargs"]["stacked_obs"] = config["rl_agent"][
            "frame_stacking"
        ]["enabled"]
        ppo_kwargs["policy_kwargs"] = policy_kwargs
    elif issubclass(agent_description, ActorCriticPolicy):
        raise NotImplementedError("ActorCriticPolicy not implemented yet!")
        ppo_kwargs["policy"] = agent_description
    else:
        arch_name = config["rl_agent"]["architecture_name"]
        raise TypeError(
            f"Registered agent class {arch_name} is neither of type"
            "'BaseAgent' or 'ActorCriticPolicy'!"
        )

    is_lstm = "LSTM" in agent_description.type.name
    return RecurrentPPO(**ppo_kwargs) if is_lstm else PPO(**ppo_kwargs)


sys.modules["rl_agent"] = sys.modules["rosnav"]
sys.modules["rl_utils.rl_utils.utils"] = sys.modules["rosnav.utils"]


def load_model(config: dict, train_env: VecEnv, PATHS: dict) -> PPO:
    agent_name = config["agent_name"]
    checkpoint = config["rl_agent"]["checkpoint"]
    possible_agent_names = [
        checkpoint,
        "best_model",
        "last_model",
        f"{agent_name}",
        "model",
    ]

    for name in possible_agent_names:
        target_path = os.path.join(PATHS["model"], f"{checkpoint}.zip")
        if os.path.isfile(target_path):
            rospy.loginfo(f"Loading model from {target_path}")
            model = PPO.load(os.path.join(PATHS["model"], name), train_env)
            break

    if not model:
        raise FileNotFoundError(
            f"Could not find model file for agent {agent_name}!"
            "You might need to change the 'checkpoint' in the config file "
            "accordingly."
        )

    update_hyperparam_model(model, PATHS, config)
    return model


def init_callbacks(
    config: dict, train_env: VecEnv, eval_env: VecEnv, paths
) -> EvalCallback:
    # threshold settings for training curriculum
    # type can be either 'succ' or 'rew'
    curriculum_cfg = config["callbacks"]["training_curriculum"]
    stop_train_cfg = config["callbacks"]["stop_training"]
    periodic_eval_cfg = config["callbacks"]["periodic_eval"]

    trainstage_cb = InitiateNewTrainStage(
        n_envs=config["n_envs"],
        treshhold_type=curriculum_cfg["threshold_type"],
        upper_threshold=curriculum_cfg["upper_threshold"],
        lower_threshold=curriculum_cfg["lower_threshold"],
        activated="staged" in config["tm_modules"],
        verbose=1,
    )

    # stop training on reward threshold callback
    stoptraining_cb = StopTrainingOnRewardThreshold(
        treshhold_type=stop_train_cfg["threshold_type"],
        threshold=stop_train_cfg["threshold"],
        verbose=1,
    )

    # evaluation settings
    # n_eval_episodes: number of episodes to evaluate agent on
    # eval_freq: evaluate the agent every eval_freq train timesteps
    eval_cb = EvalCallback(
        eval_env=eval_env,
        train_env=train_env,
        n_eval_episodes=periodic_eval_cfg["n_eval_episodes"],
        eval_freq=periodic_eval_cfg["eval_freq"],
        log_path=paths["eval"],
        best_model_save_path=None if config["debug_mode"] else paths["model"],
        deterministic=True,
        callback_on_eval_end=trainstage_cb,
        callback_on_new_best=stoptraining_cb,
    )

    return eval_cb
