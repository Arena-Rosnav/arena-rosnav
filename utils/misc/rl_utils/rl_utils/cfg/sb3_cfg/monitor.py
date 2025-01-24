from pydantic import BaseModel
from typing import Optional


class EpisodeLoggingCfg(BaseModel):
    last_n_episodes: int = 20
    record_actions: bool = True


class MonitoringCfg(BaseModel):
    wandb: bool = True
    training_metrics: Optional[bool] = True
    episode_logging: Optional[EpisodeLoggingCfg] = EpisodeLoggingCfg()
    eval_metrics: Optional[bool] = False  # eval_log
