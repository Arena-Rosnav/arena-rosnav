import rl_utils.cfg as cfg
from rl_utils.tools.constants import TRAINING_CONSTANTS

from .general import load_config


def load_training_config(config_name: str) -> cfg.TrainingCfg:
    raw_config = load_config(TRAINING_CONSTANTS.PATHS.TRAINING_CONFIGS(config_name))
    return cfg.TrainingCfg.model_validate(raw_config, strict=True, from_attributes=True)
