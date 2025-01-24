from pydantic import BaseModel
from typing import Optional


class NormalizationCfg(BaseModel):
    load_from: Optional[str] = None
    norm_obs: Optional[bool] = False
    norm_reward: Optional[bool] = False
    clip_obs: Optional[float] = 30.0
    clip_reward: Optional[float] = 30.0
    gamma: Optional[float] = 0.99
