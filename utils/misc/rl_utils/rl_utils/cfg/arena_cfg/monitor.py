from pydantic import BaseModel, Field
from typing import Optional, List


class EpisodeLoggingCfg(BaseModel):
    last_n_episodes: int = 20
    record_actions: bool = True

class WandbCfg(BaseModel):
    project_name: str = Field(
        default="Arena-RL", 
        title="Project Name", 
        description="Name of the Weights & Biases project.", 
        exclude=None
    )
    run_name: Optional[str] = None
    group: Optional[str] = None
    tags: Optional[List[str]] = None

class MonitoringCfg(BaseModel):
    wandb: WandbCfg = WandbCfg()
    training_metrics: Optional[bool] = True
    episode_logging: Optional[EpisodeLoggingCfg] = EpisodeLoggingCfg()
    eval_metrics: Optional[bool] = False  # eval_log
