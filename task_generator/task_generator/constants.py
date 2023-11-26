from enum import Enum
import random
from typing import Any, Callable, Optional

import rospy
from task_generator.shared import Namespace


class Defaults:
    class task_config:
        no_of_episodes = 5


class Constants:
    GOAL_REACHED_TOLERANCE = 1.0
    TIMEOUT = 3.0 * 60  # 3 min
    WAIT_FOR_SERVICE_TIMEOUT = 60  # 5 secs
    MAX_RESET_FAIL_TIMES = 10

    class ObstacleManager:
        DYNAMIC_OBSTACLES = 15
        STATIC_OBSTACLES = 15
        INTERACTIVE_OBSTACLES = 15

        OBSTACLE_MAX_RADIUS = 0.6

    class RobotManager:
        SPAWN_ROBOT_SAFE_DIST = 0.25

    class Simulator(Enum):
        FLATLAND = "flatland"
        GAZEBO = "gazebo"

    class ArenaType(Enum):
        TRAINING = "training"
        DEPLOYMENT = "deployment"

    class EntityManager(Enum):
        PEDSIM = "pedsim"
        FLATLAND = "flatland"

    class TaskMode(Enum):
        RANDOM = "random"
        STAGED = "staged"
        SCENARIO = "scenario"
        PARAMETRIZED = "parametrized"
        DYNAMIC_MAP_RANDOM = "dynamic_map_random"
        DYNAMIC_MAP_STAGED = "dynamic_map_staged"
        GUIDED = "guided"

    class MapGenerator:
        NODE_NAME = "map_generator"
        MAP_FOLDER_NAME = "dynamic_map"

    class Random:
        MIN_DYNAMIC_OBS = 0
        MAX_DYNAMIC_OBS = 0
        MIN_STATIC_OBS = 0
        MAX_STATIC_OBS = 0
        MIN_INTERACTIVE_OBS = 0
        MAX_INTERACTIVE_OBS = 0

    class Scenario:
        RESETS_DEFAULT = 5

    PLUGIN_FULL_RANGE_LASER = {
        "type": "Laser",
        "name": "full_static_laser",
        "frame": "full_laser",
        "topic": "full_scan",
        "body": "link_base",
        "broadcast_tf": "true",
        "origin": [0, 0, 0],
        "range": 2.0,
        "angle": {"min": -3.14, "max": 3.14, "increment": 0.01745},
        "noise_std_dev": 0.0,
        "update_rate": 10,
    }


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
pedsim_ns = Namespace("task_generator_node/configuration/pedsim/default_actor_config")


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
            max(lo,
                random.normalvariate((hi + lo) / 2, (hi - lo) / 6)
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
    TALKING_AND_WALKING_PROBABILITY = lp("TALKING_AND_WALKING_PROBABILITY", 0.0)
    REQUESTING_SERVICE_PROBABILITY = lp("REQUESTING_SERVICE_PROBABILITY", 0.0)
    REQUESTING_GUIDE_PROBABILITY = lp("REQUESTING_GUIDE_PROBABILITY", 0.0)
    REQUESTING_FOLLOWER_PROBABILITY = lp("REQUESTING_FOLLOWER_PROBABILITY", 0.0)
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
