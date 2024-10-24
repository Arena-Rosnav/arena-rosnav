from typing import Optional

from pydantic import BaseModel, model_validator
from rosnav_rl.cfg.stable_baselines3.ppo import PPO_Cfg
from tools.model_utils import check_batch_size

from .callbacks import CallbacksCfg
from .general import GeneralCfg
from .monitor import MonitoringCfg
from .normalization import NormalizationCfg
from .profile import ProfilingCfg
from .robot import RobotCfg
from .task import TaskCfg


class SB3Cfg(BaseModel):
    general: GeneralCfg = GeneralCfg()
    monitoring: MonitoringCfg = MonitoringCfg()
    task: Optional[TaskCfg] = TaskCfg()
    normalization: Optional[NormalizationCfg] = None
    algorithm: PPO_Cfg = PPO_Cfg()
    callbacks: CallbacksCfg = CallbacksCfg()
    profiling: Optional[ProfilingCfg] = None
    robot: Optional[RobotCfg] = RobotCfg()

    @model_validator(mode="after")
    def check_batch_size(self):
        check_batch_size(
            self.general.n_envs,
            batch_size=self.algorithm.total_batch_size,
            mn_batch_size=self.algorithm.batch_size,
        )
        return self

    @model_validator(mode="after")
    def check_n_steps(self):
        if self.algorithm.n_steps is None:
            self.algorithm.n_steps = int(
                self.algorithm.batch_size / self.general.n_envs
            )
        return self
