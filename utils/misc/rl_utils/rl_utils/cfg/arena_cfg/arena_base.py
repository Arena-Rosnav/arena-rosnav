from typing import Optional

from pydantic import BaseModel, field_validator

from .general import GeneralCfg
from .monitor import MonitoringCfg
from .profile import ProfilingCfg
from .robot import RobotCfg
from .task import TaskCfg


class ArenaBaseCfg(BaseModel):
    general: Optional[GeneralCfg] = GeneralCfg()
    monitoring: Optional[MonitoringCfg] = MonitoringCfg()
    task: Optional[TaskCfg] = TaskCfg()
    profiling: Optional[ProfilingCfg] = None
    robot: Optional[RobotCfg] = RobotCfg()

    @field_validator(
        "general",
        "monitoring",
        "task",
        "robot",
        mode="after",
    )
    @classmethod
    def check_attr_none(cls, v, values):
        if v is None:
            raise ValueError(f"{v} cannot be None")
        return v
