from typing import TYPE_CHECKING, List

from rl_utils.stable_baselines3.eval_callbacks.staged_train_callback import (
    InitiateNewTrainStage,
)
from rosnav_rl.utils.stable_baselines3.callbacks import (
    RosnavEvalCallback,
    StopTrainingOnRewardThreshold,
    StopTrainingOnSuccessThreshold,
)
from stable_baselines3.common.vec_env import VecEnv

if TYPE_CHECKING:
    from rl_utils.cfg import CallbacksCfg


def init_sb3_callbacks(
    eval_env: VecEnv,
    n_envs: int,
    tm_modules: List[str],
    model_save_path: str,
    eval_log_path: str,
    callback_cfg: "CallbacksCfg",
    debug_mode: bool,
) -> RosnavEvalCallback:
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

    stoptraining_cls = (
        StopTrainingOnRewardThreshold
        if stop_train_cfg.threshold_type == "rew"
        else StopTrainingOnSuccessThreshold
    )
    # stop training on reward threshold callback
    stoptraining_cb = stoptraining_cls(
        stop_train_cfg.threshold,
        verbose=stop_train_cfg.verbose,
    )

    # evaluation settings
    # n_eval_episodes: number of episodes to evaluate agent on
    # eval_freq: evaluate the agent every eval_freq train timesteps
    eval_cb = RosnavEvalCallback(
        eval_env=eval_env,
        n_eval_episodes=periodic_eval_cfg.n_eval_episodes,
        eval_freq=periodic_eval_cfg.eval_freq,
        log_path=eval_log_path,
        best_model_save_path=None if debug_mode else model_save_path,
        deterministic=True,
        callback_on_eval_end=trainstage_cb,
        callback_on_new_best=stoptraining_cb,
    )
    return eval_cb
