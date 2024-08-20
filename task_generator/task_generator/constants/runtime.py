import dataclasses
from typing import Any, Callable, Optional
import numpy as np
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult 
from task_generator.shared import Namespace
from task_generator import TASKGEN_NODE  # Use the global TASKGEN_NODE

from . import Constants

@dataclasses.dataclass
class TaskConfig_General:
    WAIT_FOR_SERVICE_TIMEOUT: float = Constants.get_default("TIMEOUT_WAIT_FOR_SERVICE")
    MAX_RESET_FAIL_TIMES: int = Constants.get_default("MAX_RESET_FAIL_TIMES")
    RNG: np.random.Generator = np.random.default_rng(1)
    DESIRED_EPISODES: float = Constants.get_default("EPISODES")

@dataclasses.dataclass
class TaskConfig_Robot:
    GOAL_TOLERANCE_RADIUS: float = Constants.get_default("GOAL_RADIUS")
    GOAL_TOLERANCE_ANGLE: float = Constants.get_default("GOAL_TOLERANCE_ANGLE")
    SPAWN_ROBOT_SAFE_DIST: float = Constants.get_default("SPAWN_ROBOT_SAFE_DIST")
    TIMEOUT: float = Constants.get_default("TIMEOUT")

@dataclasses.dataclass
class TaskConfig_Obstacles:
    OBSTACLE_MAX_RADIUS: float = Constants.get_default("OBSTACLE_MAX_RADIUS")

@dataclasses.dataclass
class TaskConfig:
    General: TaskConfig_General = TaskConfig_General()
    Robot: TaskConfig_Robot = TaskConfig_Robot()
    Obstacles: TaskConfig_Obstacles = TaskConfig_Obstacles()

Config = TaskConfig()

# Use the global TASKGEN_NODE for parameter declaration and fetching initial values
TASKGEN_NODE.declare_parameter('timeout_wait_for_service', Config.General.WAIT_FOR_SERVICE_TIMEOUT)
TASKGEN_NODE.declare_parameter('max_reset_fail_times', Config.General.MAX_RESET_FAIL_TIMES)
TASKGEN_NODE.declare_parameter('goal_radius', Config.Robot.GOAL_TOLERANCE_RADIUS)
TASKGEN_NODE.declare_parameter('goal_tolerance_angle', Config.Robot.GOAL_TOLERANCE_ANGLE)
TASKGEN_NODE.declare_parameter('spawn_robot_safe_dist', Config.Robot.SPAWN_ROBOT_SAFE_DIST)
TASKGEN_NODE.declare_parameter('timeout', Config.Robot.TIMEOUT)
TASKGEN_NODE.declare_parameter('obstacle_max_radius', Config.Obstacles.OBSTACLE_MAX_RADIUS)
TASKGEN_NODE.declare_parameter('episodes', Config.General.DESIRED_EPISODES)

# Fetch the initial parameter values
Config.General.WAIT_FOR_SERVICE_TIMEOUT = TASKGEN_NODE.get_parameter('timeout_wait_for_service').get_parameter_value().double_value
Config.General.MAX_RESET_FAIL_TIMES = TASKGEN_NODE.get_parameter('max_reset_fail_times').get_parameter_value().integer_value
Config.Robot.GOAL_TOLERANCE_RADIUS = TASKGEN_NODE.get_parameter('goal_radius').get_parameter_value().double_value
Config.Robot.GOAL_TOLERANCE_ANGLE = TASKGEN_NODE.get_parameter('goal_tolerance_angle').get_parameter_value().double_value
Config.Robot.SPAWN_ROBOT_SAFE_DIST = TASKGEN_NODE.get_parameter('spawn_robot_safe_dist').get_parameter_value().double_value
Config.Robot.TIMEOUT = TASKGEN_NODE.get_parameter('timeout').get_parameter_value().double_value
Config.Obstacles.OBSTACLE_MAX_RADIUS = TASKGEN_NODE.get_parameter('obstacle_max_radius').get_parameter_value().double_value
Config.General.DESIRED_EPISODES = TASKGEN_NODE.get_parameter('episodes').get_parameter_value().double_value

# Define the parameter callback function
def parameter_callback(params: list[Parameter]):
    global Config
    for param in params:
        if param.name == 'timeout_wait_for_service':
            Config.General.WAIT_FOR_SERVICE_TIMEOUT = param.value
        elif param.name == 'max_reset_fail_times':
            Config.General.MAX_RESET_FAIL_TIMES = param.value
        elif param.name == 'goal_radius':
            Config.Robot.GOAL_TOLERANCE_RADIUS = param.value
        elif param.name == 'goal_tolerance_angle':
            Config.Robot.GOAL_TOLERANCE_ANGLE = param.value
        elif param.name == 'spawn_robot_safe_dist':
            Config.Robot.SPAWN_ROBOT_SAFE_DIST = param.value
        elif param.name == 'timeout':
            Config.Robot.TIMEOUT = param.value
        elif param.name == 'obstacle_max_radius':
            Config.Obstacles.OBSTACLE_MAX_RADIUS = param.value
        elif param.name == 'episodes':
            Config.General.DESIRED_EPISODES = float(param.value)

    return SetParametersResult(successful=True)

# Register the parameter callback
TASKGEN_NODE.add_on_set_parameters_callback(parameter_callback)

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

def lp(parameter: str, fallback: Any) -> Callable[[Optional[Any]], Any]:
    """
    Load Pedsim param
    """
    val = fallback
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
