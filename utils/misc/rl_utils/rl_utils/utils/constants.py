from enum import Enum


class Simulator(Enum):
    FLATLAND = "flatland"
    GAZEBO = "gazebo"
    UNITY = "unity"


class ArenaType(Enum):
    TRAINING = "training"
    DEPLOYMENT = "deployment"


class EntityManager(Enum):
    PEDSIM = "pedsim"
    FLATLAND = "flatland"
    CROWDSIM = "crowdsim"


class MapGenerator:
    NODE_NAME = "map_generator"
    MAP_FOLDER_NAME = "dynamic_map"
