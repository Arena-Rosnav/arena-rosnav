from typing import Tuple

import rl_utils.cfg as cfg

from tools.constants import TRAINING_CONSTANTS

from .general import load_config


def load_training_config(config_name: str) -> cfg.SB3TrainingCfg:
    raw_config = load_config(TRAINING_CONSTANTS.PATHS.TRAINING_CONFIGS(config_name))
    return cfg.SB3TrainingCfg.model_validate(
        raw_config, strict=True, from_attributes=True
    )


def unpack_framework_config(
    framework_cfg: cfg.SB3Cfg,
) -> Tuple[
    cfg.GeneralCfg,
    cfg.CallbacksCfg,
    cfg.MonitoringCfg,
    cfg.TaskCfg,
    cfg.RobotCfg,
    cfg.NormalizationCfg,
    cfg.ProfilingCfg,
]:
    return (
        framework_cfg.general,
        framework_cfg.callbacks,
        framework_cfg.monitoring,
        framework_cfg.task,
        framework_cfg.robot,
        framework_cfg.normalization,
        framework_cfg.profiling,
    )
