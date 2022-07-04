import rospy
import rospkg
import os
import yaml
from gym import spaces
import numpy as np


def get_default_hyperparams_path():
    return os.path.join(
        rospkg.RosPack().get_path("training"),
        "configs",
        "hyperparameters",
        "default.json",
    )


def get_trained_models_path():
    return os.path.join(
        rospkg.RosPack().get_path("rosnav"), "agents"
    )


def stack_spaces(*ss):
    low = []
    high = []
    for space in ss:
        low.extend(space.low.tolist())
        high.extend(space.high.tolist())
    return spaces.Box(np.array(low).flatten(), np.array(high).flatten())