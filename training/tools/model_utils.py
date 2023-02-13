import os
import sys
from typing import Union, Type

import wandb
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    EvalCallback,
    StopTrainingOnRewardThreshold,
)
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.vec_env.base_vec_env import VecEnv
from stable_baselines3.common.utils import configure_logger

from rosnav.model.base_agent import BaseAgent
from tools.staged_train_callback import InitiateNewTrainStage


def setup_wandb(config: dict, agent: PPO) -> None:
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
        batch_size > mn_batch_size
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
    print("(no change -> no print below")

    ppo_params = config["rl_agent"]["ppo"]
    update(model, "batch_size", ppo_params["m_batch_size"])
    update(model, "gamma", ppo_params["gamma"])
    update(model, "n_steps", ppo_params["n_steps"])
    update(model, "ent_coef", ppo_params["ent_coef"])
    update(model, "learning_rate", ppo_params["learning_rate"])
    update(model, "vf_coef", ppo_params["vf_coef"])
    update(model, "max_grad_norm", ppo_params["max_grad_norm"])
    update(model, "gae_lambda", ppo_params["gae_lambda"])
    update(model, "n_epochs", ppo_params["n_epochs"])
    """
    if model.clip_range != params['clip_range']:
        model.clip_range = params['clip_range']
    """
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
    if not model.tensorboard_log and (
        not config["debug_mode"] and config["monitoring"]["use_wandb"]
    ):
        model.tensorboard_log = PATHS["tb"]
        configure_logger(1, PATHS["tb"], "run", False)

    print("--------------------------------\n")


def get_ppo_instance(
    config: dict,
    train_env: VecEnv,
    PATHS: dict,
    AgentFactory,
) -> PPO:
    new_model: bool = (
        config["rl_agent"]["architecture_name"] and not config["rl_agent"]["resume"]
    )
    model = (
        instantiate_new_model(config, train_env, PATHS, AgentFactory)
        if new_model
        else load_model(config, train_env, PATHS)
    )

    wandb_logging: bool = not config["debug_mode"] and config["monitoring"]["use_wandb"]
    if wandb_logging and not new_model:
        setup_wandb(config, model)
    return model


def instantiate_new_model(
    config: dict, train_env: VecEnv, PATHS: dict, AgentFactory
) -> PPO:
    agent: Union[Type[BaseAgent], Type[ActorCriticPolicy]] = AgentFactory.instantiate(
        config["rl_agent"]["architecture_name"]
    )

    ppo_config = config["rl_agent"]["ppo"]
    ppo_kwargs = {
        "env": train_env,
        "gamma": ppo_config["gamma"],
        "n_steps": ppo_config["n_steps"],
        "ent_coef": ppo_config["ent_coef"],
        "learning_rate": ppo_config["learning_rate"],
        "vf_coef": ppo_config["vf_coef"],
        "max_grad_norm": ppo_config["max_grad_norm"],
        "gae_lambda": ppo_config["gae_lambda"],
        "batch_size": ppo_config["m_batch_size"],
        "n_epochs": ppo_config["n_epochs"],
        "clip_range": ppo_config["clip_range"],
        "tensorboard_log": PATHS["tb"],
        # "use_wandb": False if config["debug_mode"] else config["monitoring"]["use_wandb"],
        "verbose": 1,
    }

    if isinstance(agent, BaseAgent):
        ppo_kwargs["policy"] = agent.type.value
        ppo_kwargs["policy_kwargs"] = agent.get_kwargs()
    elif issubclass(agent, ActorCriticPolicy):
        ppo_kwargs["policy"] = agent
    else:
        arch_name = config["rl_agent"]["architecture_name"]
        raise TypeError(
            f"Registered agent class {arch_name} is neither of type"
            "'BaseAgent' or 'ActorCriticPolicy'!"
        )

    return PPO(**ppo_kwargs)


def load_model(config: dict, train_env: VecEnv, PATHS: dict) -> PPO:
    agent_name = config["agent_name"]
    possible_agent_names = [f"{agent_name}", "best_model", "model"]

    for name in possible_agent_names:
        if os.path.isfile(os.path.join(PATHS["model"], f"{name}.zip")):
            model = PPO.load(os.path.join(PATHS["model"], name), train_env)
            break

    if not model:
        raise FileNotFoundError(
            f"Could not find model file for agent {agent_name}!"
            "You might need to change the 'agent_name' in the config file "
            "according to the name of the parent directory of the desired model."
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
        task_mode=config["task_mode"],
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
