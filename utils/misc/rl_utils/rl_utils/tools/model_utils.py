import re
from typing import List, TYPE_CHECKING

import rospy
import torch

from rl_utils.stable_baselines3.eval_callbacks.staged_train_callback import (
    InitiateNewTrainStage,
)

from sb3_contrib import RecurrentPPO
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    EvalCallback,
    StopTrainingOnRewardThreshold,
)
from stable_baselines3.common.utils import configure_logger, constant_fn
from stable_baselines3.common.vec_env.base_vec_env import VecEnv

import wandb

if TYPE_CHECKING:
    from rl_utils.cfg import TrainingCfg

    
def setup_wandb(run_name: str=None, group: str = None, config: "TrainingCfg"=None, agent_id:str=None, to_watch: List[torch.nn.Module] = []) -> None:
    """
    Set up Weights and Biases (wandb) for tracking and visualizing training.

    This function logs into wandb, initializes a new run with the given training
    configuration, and sets up monitoring for TensorBoard and Gym environments.
    It also watches the policy model for logging gradients and parameters.

    Args:
        train_cfg (TrainingCfg): The training configuration object containing
            settings and parameters for the training session.
        rl_model (RL_Model): The reinforcement learning model to be tracked
            and monitored by wandb.

    Returns:
        None
    """
    wandb.login()
    wandb.init(
        name=run_name if run_name else config.arena_cfg.monitoring.wandb.run_name,
        group=group if group else config.arena_cfg.monitoring.wandb.group,
        project=config.arena_cfg.monitoring.wandb.project_name,
        tags=config.arena_cfg.monitoring.wandb.tags,
        entity=None,
        sync_tensorboard=True,
        monitor_gym=False,
        save_code=False,
        config=config.model_dump(),
        id=agent_id,
    )
    for module in to_watch:
        wandb.watch(module, log_graph=True)


def check_batch_size(n_envs: int, batch_size: int, mn_batch_size: int) -> None:
    """
    Validates the batch size against the number of environments and mini batch size.

    Parameters:
    n_envs (int): Number of environments.
    batch_size (int): The total batch size.
    mn_batch_size (int): The mini batch size.

    Raises:
    ValueError: If any of the following conditions are met:
        - The mini batch size is greater than the batch size.
        - The batch size is not divisible by the mini batch size.
        - The batch size is not divisible by the number of environments.
    """
    errors = []

    if batch_size < mn_batch_size:
        errors.append(
            f"Mini batch size {mn_batch_size} is bigger than batch size {batch_size}"
        )

    if batch_size % mn_batch_size != 0:
        errors.append(
            f"Batch size {batch_size} isn't divisible by mini batch size {mn_batch_size}"
        )

    if batch_size % n_envs != 0:
        errors.append(f"Batch size {batch_size} isn't divisible by n_envs {n_envs}")

    if errors:
        raise ValueError(" | ".join(errors))


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

    ppo_params: dict = config["rl_agent"]["ppo"]
    update(model, "batch_size", ppo_params["m_batch_size"])
    update(model, "gamma", ppo_params["gamma"])
    update(model, "n_steps", ppo_params["n_steps"])
    update(model, "ent_coef", ppo_params["ent_coef"])
    update(model, "vf_coef", ppo_params["vf_coef"])
    update(model, "max_grad_norm", ppo_params["max_grad_norm"])
    update(model, "gae_lambda", ppo_params["gae_lambda"])
    update(model, "n_epochs", ppo_params["n_epochs"])
    update(model, "clip_range", constant_fn(ppo_params["clip_range"]))
    update(model, "target_kl", ppo_params.get("target_kl", None))

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

    if not isinstance(model, RecurrentPPO):
        model._setup_rollout_buffer()
    else:
        init_rppo_rollout_buffer(model)

    print("--------------------------------\n")


def init_sb3_callbacks(
    train_env: VecEnv,
    eval_env: VecEnv,
    n_envs: int,
    tm_modules: List[str],
    model_save_path: str,
    eval_log_path: str,
    callback_cfg,
    debug_mode: bool,
) -> EvalCallback:
    """
    Initialize Stable Baselines3 (SB3) callbacks for training and evaluation.

    Args:
        train_env (VecEnv): The training environment.
        eval_env (VecEnv): The evaluation environment.
        n_envs (int): Number of environments.
        tm_modules (List[str]): List of training modules.
        model_save_path (str): Path to save the model.
        eval_log_path (str): Path to save evaluation logs.
        callback_cfg (CallbacksCfg): Configuration for callbacks.
        debug_mode (bool): If True, disables model saving for debugging purposes.

    Returns:
        EvalCallback: The evaluation callback configured with the specified settings.
    """
    # threshold settings for training curriculum
    # type can be either 'succ' or 'rew'
    curriculum_cfg = callback_cfg.training_curriculum
    stop_train_cfg = callback_cfg.stop_training_on_threshold
    periodic_eval_cfg = callback_cfg.periodic_evaluation

    trainstage_cb = InitiateNewTrainStage(
        n_envs=n_envs,
        treshhold_type=curriculum_cfg.threshold_type,
        upper_threshold=curriculum_cfg.upper_threshold,
        lower_threshold=curriculum_cfg.lower_threshold,
        activated="staged" in tm_modules,
        verbose=1,
    )

    # stop training on reward threshold callback
    stoptraining_cb = StopTrainingOnRewardThreshold(
        treshhold_type=stop_train_cfg.threshold_type,
        threshold=stop_train_cfg.threshold,
        verbose=stop_train_cfg.verbose,
    )

    # evaluation settings
    # n_eval_episodes: number of episodes to evaluate agent on
    # eval_freq: evaluate the agent every eval_freq train timesteps
    eval_cb = EvalCallback(
        eval_env=eval_env,
        train_env=train_env,
        n_eval_episodes=periodic_eval_cfg.n_eval_episodes,
        eval_freq=periodic_eval_cfg.eval_freq,
        log_path=eval_log_path,
        best_model_save_path=None if debug_mode else model_save_path,
        deterministic=True,
        callback_on_eval_end=trainstage_cb,
        callback_on_new_best=stoptraining_cb,
    )
    return eval_cb
