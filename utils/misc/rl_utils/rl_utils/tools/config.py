from dataclasses import dataclass, fields
from typing import Any, Dict, Generic, Type, TypeVar, get_type_hints

import rl_utils.cfg as cfg
import rosnav_rl.cfg as rosnav_cfg
from pydantic import BaseModel

from rl_utils.tools.constants import TRAINING_CONSTANTS

from .general import load_config

T = TypeVar("T", bound=BaseModel)


def load_training_config(config_name: str) -> cfg.TrainingCfg:
    raw_config = load_config(TRAINING_CONSTANTS.PATHS.TRAINING_CONFIGS(config_name))
    return cfg.TrainingCfg.model_validate(raw_config, strict=True, from_attributes=True)


ConfigT = TypeVar("ConfigT")
FieldsT = TypeVar("FieldsT")


class ConfigManager(Generic[ConfigT, FieldsT]):
    """Base configuration manager with type hints."""

    _config: cfg.TrainingCfg
    fields: FieldsT

    def __init__(self, config: cfg.TrainingCfg, fields_class: Type[FieldsT]) -> None:
        """Initialize config manager with typed fields."""
        self._config = config
        # Initialize fields with None values while maintaining type hints
        self.fields = fields_class(
            **{field.name: None for field in fields(fields_class)}
        )
        self._initialize()

    def _initialize(self) -> None:
        """Dynamically initialize configuration based on typed fields."""
        field_types = get_type_hints(self.fields.__class__)

        # Special case for agent_cfg
        if "agent" in field_types:
            setattr(self.fields, "agent", self._config.agent_cfg)
            field_types.pop("agent")

        # Initialize remaining fields from framework_cfg
        for field_name, field_type in field_types.items():
            setattr(
                self.fields, field_name, getattr(self._config.framework_cfg, field_name)
            )


@dataclass
class SB3ConfigFields:
    """SB3 configuration fields."""

    general: cfg.GeneralCfg
    monitoring: cfg.MonitoringCfg
    task: cfg.TaskCfg
    normalization: cfg.NormalizationCfg
    callbacks: cfg.CallbacksCfg
    profiling: cfg.ProfilingCfg
    robot: cfg.RobotCfg
    agent: rosnav_cfg.AgentCfg


class SB3ConfigManager(ConfigManager[cfg.TrainingCfg, SB3ConfigFields]):
    """Type-hinted SB3 configuration manager."""

    def __init__(self, config: cfg.TrainingCfg) -> None:
        super().__init__(config, SB3ConfigFields)
