#!/usr/bin/env python

from rl_utils.cfg import TrainingCfg
from rl_utils.tools.argsparser import parse_training_args
from rl_utils.tools.config import load_training_config
from rl_utils.trainer import DreamerV3Trainer, StableBaselines3Trainer
import rosnav_rl.utils.type_aliases as type_aliases


def get_trainer(config: TrainingCfg):
    """Create and return the appropriate trainer based on the framework specified in config."""
    framework = config.agent_cfg.framework.name
    
    if framework == type_aliases.SupportedRLFrameworks.DREAMER_V3.value:
        return DreamerV3Trainer(config)
    elif framework == type_aliases.SupportedRLFrameworks.STABLE_BASELINES3.value:
        return StableBaselines3Trainer(config)
    else:
        raise ValueError(f"Unsupported framework: {framework}")


def main():
    args, _ = parse_training_args()
    config = load_training_config(args.config)
    
    trainer = get_trainer(config)
    trainer.train()


if __name__ == "__main__":
    main()
