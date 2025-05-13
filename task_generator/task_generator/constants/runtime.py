import os
import typing

import numpy as np
import rclpy
from arena_rclpy_mixins.ROSParamServer import ROSParamServer

from ..utils.arena import get_simulation_setup_path
from . import Constants


def Configuration(server: ROSParamServer):

    def _positive_or_inf(v: typing.Any) -> float:
        return v if v >= 0 else float('inf')

    class Config:
        """
        Combined Task Config
        """

        class Arena:
            """
            Formerly arena.py.
            """

            SIMULATOR = server.ROSParam[Constants.Simulator](
                'simulator',
                Constants.Simulator.DUMMY.value,
                parse=Constants.Simulator
            )

            WORLD = server.ROSParam[str](
                'world',
                type_=rclpy.Parameter.Type.STRING,
            )

            @classmethod
            def get_world_path(cls, world: str | None = None) -> str:
                """
                Get absolute path of current world directory.
                """
                if world is None:
                    world = str(cls.WORLD.value)
                return os.path.join(
                    get_simulation_setup_path(),
                    'worlds',
                    world,
                )

            ENTITY_MANAGER = server.ROSParam[Constants.EntityManager](
                'entity_manager',
                Constants.EntityManager.DUMMY.value,
                parse=Constants.EntityManager
            )

        class General:
            """
            General Task Configuration
            """

            WAIT_FOR_SERVICE_TIMEOUT = server.ROSParam[float](
                'timeout_wait_for_service', 30)

            MAX_RESET_FAIL_TIMES = server.ROSParam[int](
                'max_reset_fail_times', 10)

            RNG = server.ROSParam[np.random.Generator](
                'rng',
                -1,
                parse=lambda x: np.random.default_rng(x) if x >= 0 else np.random.default_rng()
            )
            DESIRED_EPISODES = server.ROSParam[float](
                'episodes',
                -1,
                parse=_positive_or_inf
            )

        class Obstacles:
            OBSTACLE_MAX_RADIUS = server.ROSParam[float](
                'obstacle_max_radius',
                15,
                parse=_positive_or_inf
            )

        class Robot:
            GOAL_TOLERANCE_RADIUS = server.ROSParam[float](
                'goal_tolerance_radius', 1.0)

            GOAL_TOLERANCE_ANGLE = server.ROSParam[float](
                'goal_tolerance_angle', 30.0 * np.pi / 360.)

            SPAWN_ROBOT_SAFE_DIST = server.ROSParam[float](
                'robot_safe_dist', .25)

            TIMEOUT = server.ROSParam[float](
                'timeout',
                -1,
                parse=_positive_or_inf
            )

        class TaskMode:
            TM_ROBOTS = server.ROSParam[Constants.TaskMode.TM_Robots](
                'tm_robots',
                Constants.TaskMode.TM_Robots.default().value,
                parse=Constants.TaskMode.TM_Robots
            )

            TM_OBSTACLES = server.ROSParam[Constants.TaskMode.TM_Obstacles](
                'tm_obstacles',
                Constants.TaskMode.TM_Obstacles.default().value,
                parse=Constants.TaskMode.TM_Obstacles
            )

            TM_MODULES = server.ROSParam[set[Constants.TaskMode.TM_Module]](
                'tm_modules',
                ','.join(
                    [m.value for m in Constants.TaskMode.TM_Module.default()]),
                parse=lambda x: {
                    Constants.TaskMode.TM_Module(m) for m in x.split(',') if m != ''
                }
            )

    return Config


# def lp(parameter: str, fallback: Any) -> Callable[[Optional[Any]], Any]:
#     """
#     load parameter
#     """
#     val = fallback

#     def gen():
#         return val

#     if isinstance(val, list):
#         lo, hi = val[:2]

#         def new_gen():
#             return min(
#                 hi,
#                 max(
#                     lo,
#                     Config.General.RNG.normal((hi + lo) / 2, (hi - lo) / 6)
#                 )
#             )
#         gen = new_gen

#     return lambda x: x if x is not None else gen()


# class Hunavsim:
#     VMAX = lp("VMAX", 0.3)
#     WAYPOINT_MODE = lp("WAYPOINT_MODE", 0)
#     FORCE_FACTOR_DESIRED = lp("FORCE_FACTOR_DESIRED", 1.0)
#     FORCE_FACTOR_OBSTACLE = lp("FORCE_FACTOR_OBSTACLE", 1.0)
#     FORCE_FACTOR_SOCIAL = lp("FORCE_FACTOR_SOCIAL", 5.0)
#     FORCE_FACTOR_ROBOT = lp("FORCE_FACTOR_ROBOT", 0.0)
