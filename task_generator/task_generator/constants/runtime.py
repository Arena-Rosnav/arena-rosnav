import dataclasses
from typing import Any, Callable, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from task_generator.shared import Namespace

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

def lp(parameter: str, fallback: Any) -> Callable[[Optional[Any]], Any]:
    """
    load parameter
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

class Hunavsim:
    VMAX = lp("VMAX", 0.3)
    WAYPOINT_MODE = lp("WAYPOINT_MODE", 0)
    FORCE_FACTOR_DESIRED = lp("FORCE_FACTOR_DESIRED", 1.0)
    FORCE_FACTOR_OBSTACLE = lp("FORCE_FACTOR_OBSTACLE", 1.0)
    FORCE_FACTOR_SOCIAL = lp("FORCE_FACTOR_SOCIAL", 5.0)
    FORCE_FACTOR_ROBOT = lp("FORCE_FACTOR_ROBOT", 0.0)

class TaskGenerator_ConfigNode(Node):
    def __init__(self):
        super().__init__('task_generator_config_node')

        # declare parameters and set default values 
        self.declare_parameter('timeout_wait_for_service', Config.General.WAIT_FOR_SERVICE_TIMEOUT)
        self.declare_parameter('max_reset_fail_times', Config.General.MAX_RESET_FAIL_TIMES)
        self.declare_parameter('goal_radius', Config.Robot.GOAL_TOLERANCE_RADIUS)
        self.declare_parameter('goal_tolerance_angle', Config.Robot.GOAL_TOLERANCE_ANGLE)
        self.declare_parameter('spawn_robot_safe_dist', Config.Robot.SPAWN_ROBOT_SAFE_DIST)
        self.declare_parameter('timeout', Config.Robot.TIMEOUT)
        self.declare_parameter('obstacle_max_radius', Config.Obstacles.OBSTACLE_MAX_RADIUS)
        self.declare_parameter('episodes', Config.General.DESIRED_EPISODES)

        # Fetch the initial parameter values
        Config.General.WAIT_FOR_SERVICE_TIMEOUT = self.get_parameter('timeout_wait_for_service').value
        Config.General.MAX_RESET_FAIL_TIMES = self.get_parameter('max_reset_fail_times').value
        Config.Robot.GOAL_TOLERANCE_RADIUS = self.get_parameter('goal_radius').value
        Config.Robot.GOAL_TOLERANCE_ANGLE = self.get_parameter('goal_tolerance_angle').value
        Config.Robot.SPAWN_ROBOT_SAFE_DIST = self.get_parameter('spawn_robot_safe_dist').value
        Config.Robot.TIMEOUT = self.get_parameter('timeout').value
        Config.Obstacles.OBSTACLE_MAX_RADIUS = self.get_parameter('obstacle_max_radius').value
        Config.General.DESIRED_EPISODES = self.get_parameter('episodes').value

        # set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params: list[Parameter]):
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
        return rclpy.parameter.ParameterValue()