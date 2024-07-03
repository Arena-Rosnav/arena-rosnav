import dataclasses
from enum import Enum
import enum
import math
from typing import Any, Callable, Optional

import numpy as np

import rospy
from task_generator.shared import Namespace, rosparam_get
import dynamic_reconfigure.client

class Constants:

    DEFAULT_PEDESTRIAN_MODEL = "actor1"

    TASK_GENERATOR_SERVER_NODE = Namespace("task_generator_server")

    class Simulator(Enum):
        FLATLAND = "flatland"
        GAZEBO = "gazebo"

    class ArenaType(Enum):
        TRAINING = "training"
        DEPLOYMENT = "deployment"

    class EntityManager(Enum):
        PEDSIM = "pedsim"
        FLATLAND = "flatland"
        CROWDSIM = "crowdsim"

    class TaskMode:
        @enum.unique
        class TM_Obstacles(enum.Enum):
            PARAMETRIZED = "parametrized"
            RANDOM = "random"
            SCENARIO = "scenario"

            @classmethod
            def prefix(cls, *args):
                return Namespace("tm_obstacles")(*args)

        @enum.unique
        class TM_Robots(enum.Enum):
            GUIDED = "guided"
            EXPLORE = "explore"
            RANDOM = "random"
            SCENARIO = "scenario"

            @classmethod
            def prefix(cls, *args):
                return Namespace("tm_robots")(*args)

        @enum.unique
        class TM_Module(enum.Enum):
            STAGED = "staged"
            DYNAMIC_MAP = "dynamic_map"
            CLEAR_FORBIDDEN_ZONES = "clear_forbidden_zones"
            RVIZ_UI = "rviz_ui"
            BENCHMARK = "benchmark"

            @classmethod
            def prefix(cls, *args):
                return Namespace("tm_module")(*args)

    class MapGenerator:
        NODE_NAME = "map_generator"
        MAP_FOLDER_NAME = "dynamic_map"

    PLUGIN_FULL_RANGE_LASER = {
        "type": "Laser",
        "name": "full_static_laser",
        "frame": "full_laser",
        "topic": "full_scan",
        "body": "base_link",
        "broadcast_tf": "true",
        "origin": [0, 0, 0],
        "range": 2.0,
        "angle": {"min": -3.14, "max": 3.14, "increment": 0.01745},
        "noise_std_dev": 0.0,
        "update_rate": 10,
    }


# TaskConfig


@dataclasses.dataclass
class TaskConfig_General:
    WAIT_FOR_SERVICE_TIMEOUT: float = dataclasses.field(default_factory=lambda:rosparam_get(float, "timeout_wait_for_service", 60))
    MAX_RESET_FAIL_TIMES: int = dataclasses.field(default_factory=lambda:rosparam_get(int, "max_reset_fail_times", 10))
    RNG: np.random.Generator = dataclasses.field(default_factory=lambda:np.random.default_rng())
    DESIRED_EPISODES: float = float("inf")


@dataclasses.dataclass
class TaskConfig_Robot:
    GOAL_TOLERANCE_RADIUS: float = 1.0
    GOAL_TOLERANCE_ANGLE: float = 30 * math.pi/180
    SPAWN_ROBOT_SAFE_DIST: float = 0.25
    TIMEOUT: float = float("inf")


@dataclasses.dataclass
class TaskConfig_Obstacles:
    OBSTACLE_MAX_RADIUS: float = 15


@dataclasses.dataclass
class TaskConfig:
    General: TaskConfig_General = dataclasses.field(default_factory=lambda:TaskConfig_General())
    Robot: TaskConfig_Robot = dataclasses.field(default_factory=lambda:TaskConfig_Robot())
    Obstacles: TaskConfig_Obstacles = dataclasses.field(default_factory=lambda:TaskConfig_Obstacles())


Config = TaskConfig()

def _cb_reconfigure(config):
    global Config

    Config.General.RNG=np.random.default_rng((lambda x: x if x >= 0 else None)(config["RANDOM_seed"]))
    Config.General.DESIRED_EPISODES=(lambda x: float("inf") if x<0 else x)(config["episodes"])
    
    Config.Robot.GOAL_TOLERANCE_RADIUS=config["goal_radius"]
    Config.Robot.GOAL_TOLERANCE_ANGLE=config["goal_tolerance_angle"]
    Config.Robot.TIMEOUT=(lambda x: float("inf") if x<0 else x)(config["timeout"])

dynamic_reconfigure.client.Client(
    name=Constants.TASK_GENERATOR_SERVER_NODE,
    config_callback=_cb_reconfigure
)


class FlatlandRandomModel:
    BODY = {
        "name": "base_link",
        "pose": [0, 0, 0],
        "color": [1, 0.2, 0.1, 1.0],
        "footprints": [],
    }
    FOOTPRINT = {
        "density": 1,
        "restitution": 1,
        "layers": ["all"],
        "collision": "true",
        "sensor": "false",
    }
    MIN_RADIUS = 0.2
    MAX_RADIUS = 0.6
    RANDOM_MOVE_PLUGIN = {
        "type": "RandomMove",
        "name": "RandomMove_Plugin",
        "body": "base_link",
    }
    LINEAR_VEL = 0.2
    ANGLUAR_VEL_MAX = 0.2


# no ~configuration possible because node is not fully initialized at this point
pedsim_ns = Namespace(
    "task_generator_node/configuration/pedsim/default_actor_config")


# TODO make everything dynamic_reconfigure
def lp(parameter: str, fallback: Any) -> Callable[[Optional[Any]], Any]:
    """
    load pedsim param
    """

    # load once at the start
    val = rospy.get_param(pedsim_ns(parameter), fallback)

    gen = lambda: val

    if isinstance(val, list):
        lo, hi = val[:2]
        gen = lambda: min(
            hi,
            max(
                lo,
                Config.General.RNG.normal((hi + lo) / 2, (hi - lo) / 6)
            )
        )
        # gen = lambda: random.uniform(lo, hi)

    return lambda x: x if x is not None else gen()


class Pedsim:
    VMAX = lp("VMAX", 0.3)
    START_UP_MODE = lp("START_UP_MODE", "default")
    WAIT_TIME = lp("WAIT_TIME", 0.0)
    TRIGGER_ZONE_RADIUS = lp("TRIGGER_ZONE_RADIUS", 0.0)
    CHATTING_PROBABILITY = lp("CHATTING_PROBABILITY", 0.0)
    TELL_STORY_PROBABILITY = lp("TELL_STORY_PROBABILITY", 0.0)
    GROUP_TALKING_PROBABILITY = lp("GROUP_TALKING_PROBABILITY", 0.0)
    TALKING_AND_WALKING_PROBABILITY = lp(
        "TALKING_AND_WALKING_PROBABILITY", 0.0)
    REQUESTING_SERVICE_PROBABILITY = lp("REQUESTING_SERVICE_PROBABILITY", 0.0)
    REQUESTING_GUIDE_PROBABILITY = lp("REQUESTING_GUIDE_PROBABILITY", 0.0)
    REQUESTING_FOLLOWER_PROBABILITY = lp(
        "REQUESTING_FOLLOWER_PROBABILITY", 0.0)
    MAX_TALKING_DISTANCE = lp("MAX_TALKING_DISTANCE", 5.0)
    MAX_SERVICING_RADIUS = lp("MAX_SERVICING_RADIUS", 5.0)
    TALKING_BASE_TIME = lp("TALKING_BASE_TIME", 10.0)
    TELL_STORY_BASE_TIME = lp("TELL_STORY_BASE_TIME", 0.0)
    GROUP_TALKING_BASE_TIME = lp("GROUP_TALKING_BASE_TIME", 10.0)
    TALKING_AND_WALKING_BASE_TIME = lp("TALKING_AND_WALKING_BASE_TIME", 6.0)
    RECEIVING_SERVICE_BASE_TIME = lp("RECEIVING_SERVICE_BASE_TIME", 20.0)
    REQUESTING_SERVICE_BASE_TIME = lp("REQUESTING_SERVICE_BASE_TIME", 30.0)
    FORCE_FACTOR_DESIRED = lp("FORCE_FACTOR_DESIRED", 1.0)
    FORCE_FACTOR_OBSTACLE = lp("FORCE_FACTOR_OBSTACLE", 1.0)
    FORCE_FACTOR_SOCIAL = lp("FORCE_FACTOR_SOCIAL", 5.0)
    FORCE_FACTOR_ROBOT = lp("FORCE_FACTOR_ROBOT", 0.0)
    WAYPOINT_MODE = lp("WAYPOINT_MODE", 0)
