import rl_utils.cfg as cfg
from .general import load_config

from typing import Tuple


def load_training_config(config_name: str) -> cfg.TrainingCfg:
    raw_config = load_config(config_name)
    return cfg.TrainingCfg.model_validate(raw_config, strict=True, from_attributes=True)


def unpack_framework_config(
    framework_cfg: cfg.SB3Cfg,
) -> Tuple[
    cfg.GeneralCfg,
    cfg.CallbacksCfg,
    cfg.MonitoringCfg,
    cfg.TaskCfg,
    cfg.RobotCfg,
    cfg.NormalizationCfg,
    cfg.ProfilingCfg,
]:
    return (
        framework_cfg.general,
        framework_cfg.callbacks,
        framework_cfg.monitoring,
        framework_cfg.task,
        framework_cfg.robot,
        framework_cfg.normalization,
        framework_cfg.profiling,
    )


# def init_dummy_cfg() -> cfg.TrainingCfg:
#     from rosnav_rl.cfg import AgentCfg, RewardCfg, PPO_Policy_Cfg, SB3_Model_Cfg
#     agent_cfg = AgentCfg(
#         framework=SB3_Model_Cfg(
#             model=PPO_Policy_Cfg(architecture_name="AGENT_5")
#         ),
#         reward=RewardCfg(file_name="barn"),
#     )
#     sb3_cfg = cfg.SB3Cfg()
#     sb3_cfg.general.debug_mode = True
#     train_cfg = cfg.TrainingCfg(framework_cfg=sb3_cfg, agent_cfg=agent_cfg)
#     return train_cfg
