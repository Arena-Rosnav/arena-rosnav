#!/usr/bin/env python

from rl_utils.trainer.sb3_trainer import StableBaselines3Trainer
from tools.config import load_training_config

# check dynamic parameter for training curriculum (persistent keyerror)
# enhance configuration files with field definition
# rosparam adjustments
# loading method agent level and algorithm level
# restructure folders based on framework -> algorithm
# dynamic parameter for algo and task


def main():
    config = load_training_config("training_config.yaml")
    trainer = StableBaselines3Trainer(config)
    trainer.train()


if __name__ == "__main__":
    main()
