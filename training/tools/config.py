from dataclasses import dataclass
from typing import Any, Dict, Generic, Tuple, Type, TypeVar

import rl_utils.cfg as cfg
from pydantic import BaseModel

from tools.constants import TRAINING_CONSTANTS

from .general import load_config

T = TypeVar("T", bound=BaseModel)


@dataclass
class ConfigUnpacker(Generic[T]):
    """
    A utility class for unpacking Pydantic model configurations.

    This class provides static methods to unpack Pydantic model instances into
    tuples or dictionaries, as well as to unpack specific fields from the model.

    Methods
    -------
    unpack(config: T) -> Tuple[Any, ...]
        Unpack the entire configuration into a tuple.

    unpack_dict(config: T) -> Dict[str, Any]
        Unpack the entire configuration into a dictionary.

    unpack_selected(config: T, *fields: str) -> Tuple[Any, ...]
        Unpack specific fields from the configuration into a tuple.
    """

    @staticmethod
    def unpack(config: T) -> Tuple[Any, ...]:
        """Unpack config into tuple"""
        return tuple(getattr(config, field) for field in config.__fields__)

    @staticmethod
    def unpack_dict(config: T) -> Dict[str, Any]:
        """Unpack config into dictionary"""
        return {field: getattr(config, field) for field in config.__fields__}

    @staticmethod
    def unpack_selected(config: T, *fields: str) -> Tuple[Any, ...]:
        """Unpack specific fields from config"""
        return tuple(getattr(config, field) for field in fields)


def load_training_config(config_name: str) -> cfg.TrainingCfg:
    raw_config = load_config(TRAINING_CONSTANTS.PATHS.TRAINING_CONFIGS(config_name))
    return cfg.TrainingCfg.model_validate(raw_config, strict=True, from_attributes=True)


def unpack_sb3_config(
    framework_cfg: cfg.SB3Cfg,
) -> Tuple[
    cfg.GeneralCfg,
    cfg.MonitoringCfg,
    cfg.TaskCfg,
    cfg.NormalizationCfg,
    cfg.CallbacksCfg,
    cfg.ProfilingCfg,
    cfg.RobotCfg,
]:
    unpacker = ConfigUnpacker[cfg.SB3Cfg]()
    return unpacker.unpack(framework_cfg)
