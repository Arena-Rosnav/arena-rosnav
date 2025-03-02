#!/usr/bin/env python

from rl_utils.trainer.sb3_trainer import StableBaselines3Trainer
from rl_utils.tools.argsparser import parse_training_args
from rl_utils.tools.config import load_training_config


def main():
    args, _ = parse_training_args()

    trainer = StableBaselines3Trainer(load_training_config(args.config))
    trainer.train()


if __name__ == "__main__":
    main()
