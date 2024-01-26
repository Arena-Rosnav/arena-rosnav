import itertools
import math
import random
from typing import Callable, Dict, Iterator, List
import rospy
from task_generator.constants import Constants
from task_generator.shared import (
    DynamicObstacle,
    Obstacle,
    PositionOrientation,
    PositionRadius,
    rosparam_get,
)
from task_generator.tasks.obstacles import Obstacles, TM_Obstacles
from task_generator.tasks.obstacles.utils import ITF_Obstacle
from task_generator.tasks.task_factory import TaskFactory

import dynamic_reconfigure.client
import dataclasses


@dataclasses.dataclass
class Config:
    MIN_STATIC_OBSTACLES: int
    MIN_INTERACTIVE_OBSTACLES: int
    MIN_DYNAMIC_OBSTACLES: int

    MAX_STATIC_OBSTACLES: int
    MAX_INTERACTIVE_OBSTACLES: int
    MAX_DYNAMIC_OBSTACLES: int

    MODELS_STATIC_OBSTACLES: List[str]
    MODELS_INTERACTIVE_OBSTACLES: List[str]
    MODELS_DYNAMIC_OBSTACLES: List[str]


@TaskFactory.register_obstacles(Constants.TaskMode.TM_Obstacles.RANDOM)
class TM_Random(TM_Obstacles):
    """
    Random task generator for obstacles.

    This class generates random obstacles for a task scenario.

    Attributes:
        _config (Config): Configuration object for obstacle generation.

    Methods:
        prefix(*args): Prefixes the given arguments with "scenario".
        __init__(**kwargs): Initializes the TM_Random object.
        reconfigure(config): Reconfigures the obstacle generation based on the given configuration.
        reset(**kwargs): Resets the obstacle generation with the specified parameters.

    """

    _config: Config

    @classmethod
    def prefix(cls, *args):
        return super().prefix("scenario")

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        dynamic_reconfigure.client.Client(
            name=self.NODE_CONFIGURATION, config_callback=self.reconfigure
        )

    def reconfigure(self, config):
        """
        Reconfigures the obstacle parameters based on the provided configuration.

        Args:
            config: The configuration object containing the obstacle parameters.

        Returns:
            None
        """
        self._config = Config(
            MIN_STATIC_OBSTACLES=rosparam_get(
                int, self.NODE_CONFIGURATION("RANDOM_static_min"), 0
            ),
            MIN_INTERACTIVE_OBSTACLES=rosparam_get(
                int, self.NODE_CONFIGURATION("RANDOM_interactive_min"), 0
            ),
            MIN_DYNAMIC_OBSTACLES=rosparam_get(
                int, self.NODE_CONFIGURATION("RANDOM_dynamic_min"), 0
            ),
            MAX_STATIC_OBSTACLES=rosparam_get(
                int, self.NODE_CONFIGURATION("RANDOM_static_max"), 0
            ),
            MAX_INTERACTIVE_OBSTACLES=rosparam_get(
                int, self.NODE_CONFIGURATION("RANDOM_interactive_max"), 0
            ),
            MAX_DYNAMIC_OBSTACLES=rosparam_get(
                int, self.NODE_CONFIGURATION("RANDOM_dynamic_max"), 0
            ),
            MODELS_STATIC_OBSTACLES=rosparam_get(
                str, self.NODE_CONFIGURATION("RANDOM_static_models"), ""
            ).split("/"),
            MODELS_INTERACTIVE_OBSTACLES=rosparam_get(
                str, self.NODE_CONFIGURATION("RANDOM_interactive_models"), ""
            ).split("/"),
            MODELS_DYNAMIC_OBSTACLES=rosparam_get(
                str, self.NODE_CONFIGURATION("RANDOM_dynamic_models"), ""
            ).split("/"),
        )

    def reset(self, **kwargs) -> Obstacles:
        """
        Resets the obstacle generation with the specified parameters.

        Args:
            **kwargs: Additional keyword arguments for customizing the obstacle generation.
                N_STATIC_OBSTACLES (int): Number of static obstacles.
                N_INTERACTIVE_OBSTACLES (int): Number of interactive obstacles.
                N_DYNAMIC_OBSTACLES (int): Number of dynamic obstacles.
                MODELS_STATIC_OBSTACLES (Dict[str, float]): Dictionary of static obstacle models and their weights.
                MODELS_INTERACTIVE_OBSTACLES (Dict[str, float]): Dictionary of interactive obstacle models and their weights.
                MODELS_DYNAMIC_OBSTACLES (Dict[str, float]): Dictionary of dynamic obstacle models and their weights.

        Returns:
            Tuple[List[Obstacle], List[DynamicObstacle]]: A tuple containing the generated obstacles and dynamic obstacles.

        """
        N_STATIC_OBSTACLES: int = kwargs.get(
            "N_STATIC_OBSTACLES",
            random.randint(
                self._config.MIN_STATIC_OBSTACLES, self._config.MAX_STATIC_OBSTACLES
            ),
        )
        N_INTERACTIVE_OBSTACLES: int = kwargs.get(
            "N_INTERACTIVE_OBSTACLES",
            random.randint(
                self._config.MIN_INTERACTIVE_OBSTACLES,
                self._config.MAX_INTERACTIVE_OBSTACLES,
            ),
        )
        N_DYNAMIC_OBSTACLES: int = kwargs.get(
            "N_DYNAMIC_OBSTACLES",
            random.randint(
                self._config.MIN_DYNAMIC_OBSTACLES, self._config.MAX_DYNAMIC_OBSTACLES
            ),
        )

        MODELS_STATIC_OBSTACLES: Dict[str, float] = dict.fromkeys(
            kwargs.get("MODELS_STATIC_OBSTACLES", self._config.MODELS_STATIC_OBSTACLES)
            or self._PROPS.model_loader.models,
            1,
        )
        MODELS_INTERACTIVE_OBSTACLES: Dict[str, float] = dict.fromkeys(
            kwargs.get(
                "MODELS_INTERACTIVE_OBSTACLES",
                self._config.MODELS_INTERACTIVE_OBSTACLES,
            )
            or self._PROPS.model_loader.models,
            1,
        )
        MODELS_DYNAMIC_OBSTACLES: Dict[str, float] = dict.fromkeys(
            kwargs.get(
                "MODELS_DYNAMIC_OBSTACLES", self._config.MODELS_DYNAMIC_OBSTACLES
            )
            or self._PROPS.dynamic_model_loader.models,
            1,
        )

        # rospy.logwarn(f"{MODELS_DYNAMIC_OBSTACLES}")

        def indexer() -> Callable[..., int]:
            indices: Dict[str, Iterator[int]] = dict()

            def index(model: str):
                if model not in indices:
                    indices[model] = itertools.count(1)
                return next(indices[model])

            return index

        waypoints_per_ped = 2
        points = self._PROPS.world_manager.get_positions_on_map(
            n=N_STATIC_OBSTACLES
            + N_INTERACTIVE_OBSTACLES
            + N_DYNAMIC_OBSTACLES * (1 + waypoints_per_ped),
            safe_dist=1,
        )

        _positions = [
            PositionOrientation(*pos, 2 * math.pi * random.random())
            for pos in points[
                : (N_STATIC_OBSTACLES + N_INTERACTIVE_OBSTACLES + N_DYNAMIC_OBSTACLES)
            ]
        ]
        positions = iter(_positions)

        _waypoints = [
            PositionRadius(*pos, 1)
            for pos in points[
                (N_STATIC_OBSTACLES + N_INTERACTIVE_OBSTACLES + N_DYNAMIC_OBSTACLES) :
            ]
        ]
        waypoints = iter(_waypoints)

        obstacles: List[Obstacle] = []

        # Create static obstacles
        if N_STATIC_OBSTACLES:
            index = indexer()

            obstacles += [
                ITF_Obstacle.create_obstacle(
                    self._PROPS,
                    name=f"S_{model}_{index(model)}",
                    model=self._PROPS.model_loader.bind(model),
                    position=next(positions),
                )
                for model in random.choices(
                    population=list(MODELS_STATIC_OBSTACLES.keys()),
                    weights=list(MODELS_STATIC_OBSTACLES.values()),
                    k=N_STATIC_OBSTACLES,
                )
            ]

        # Create interactive obstacles
        if N_INTERACTIVE_OBSTACLES:
            index = indexer()

            obstacles += [
                ITF_Obstacle.create_obstacle(
                    self._PROPS,
                    name=f"I_{model}_{index(model)}",
                    model=self._PROPS.model_loader.bind(model),
                    position=next(positions),
                )
                for model in random.choices(
                    population=list(MODELS_INTERACTIVE_OBSTACLES.keys()),
                    weights=list(MODELS_INTERACTIVE_OBSTACLES.values()),
                    k=N_INTERACTIVE_OBSTACLES,
                )
            ]

        # Create dynamic obstacles

        dynamic_obstacles: List[DynamicObstacle] = []

        if N_DYNAMIC_OBSTACLES:
            index = indexer()

            dynamic_obstacles += [
                ITF_Obstacle.create_dynamic_obstacle(
                    self._PROPS,
                    name=f"D_{model}_{index(model)}",
                    model=self._PROPS.dynamic_model_loader.bind(model),
                    waypoints=list(itertools.islice(waypoints, waypoints_per_ped)),
                    position=next(positions),
                )
                for model in random.choices(
                    population=list(MODELS_DYNAMIC_OBSTACLES.keys()),
                    weights=list(MODELS_DYNAMIC_OBSTACLES.values()),
                    k=N_DYNAMIC_OBSTACLES,
                )
            ]

        return obstacles, dynamic_obstacles
