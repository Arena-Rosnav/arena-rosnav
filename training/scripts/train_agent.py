#!/usr/bin/env python

from rl_utils.cfg import SB3TrainingCfg
from rl_utils.trainer.sb3_trainer import StableBaselines3Trainer
from tools.argsparser import parse_training_args


def main(config: SB3TrainingCfg):
    StableBaselines3Trainer(config)


if __name__ == "__main__":
    args, _ = parse_training_args()

    if (config_name := args.config) == "":
        raise RuntimeError("No config specified. Please specify a config file.")

    main(config_name=config_name)
