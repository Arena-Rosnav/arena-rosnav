#!/usr/bin/env python

from rl_utils.trainer.sb3_trainer import StableBaselines3Trainer
from tools.argsparser import parse_training_args
from tools.config import load_training_config


def main():
    args, _ = parse_training_args()

    trainer = StableBaselines3Trainer(load_training_config(args.config))
    trainer.train()


if __name__ == "__main__":
    main()
