from pydantic import Field

from ..arena_cfg.arena_base import ArenaBaseCfg
from .callbacks import ArenaCallbacksCfg


# Stable Baselines 3 Pipeline Configuration
class ArenaSB3Cfg(ArenaBaseCfg):
    callbacks: ArenaCallbacksCfg = Field(default_factory=ArenaCallbacksCfg, exclude=None)
