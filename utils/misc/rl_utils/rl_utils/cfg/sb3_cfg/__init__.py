from typing import Optional

from pydantic import BaseModel, field_validator, model_validator

from .callbacks import CallbacksCfg
from .general import GeneralCfg
from .monitor import MonitoringCfg
from .normalization import NormalizationCfg
from .profile import ProfilingCfg
from .robot import RobotCfg
from .task import TaskCfg


# Stable Baselines 3 Pipeline Configuration
class SB3Cfg(BaseModel):
    general: Optional[GeneralCfg] = GeneralCfg()
    monitoring: Optional[MonitoringCfg] = MonitoringCfg()
    task: Optional[TaskCfg] = TaskCfg()
    normalization: Optional[NormalizationCfg] = None
    callbacks: CallbacksCfg = CallbacksCfg()
    profiling: Optional[ProfilingCfg] = None
    robot: Optional[RobotCfg] = RobotCfg()

    @field_validator(
        "general",
        "monitoring",
        "task",
        "callbacks",
        "robot",
        mode="after",
    )
    @classmethod
    def check_attr_none(cls, v, values):
        if v is None:
            raise ValueError(f"{v} cannot be None")
        return v
