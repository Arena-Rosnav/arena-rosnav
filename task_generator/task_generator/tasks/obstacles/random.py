import functools
import itertools
import typing
from typing import Callable, Iterator

import attrs
import numpy as np
import rclpy
from arena_rclpy_mixins.ROSParamServer import ROSParamT

from task_generator.shared import (DynamicObstacle, Obstacle,
                                   PositionOrientation, PositionRadius)
from task_generator.tasks.obstacles import Obstacles, TM_Obstacles
from task_generator.tasks.obstacles.utils import ITF_Obstacle
from task_generator.utils import ModelLoader


@attrs.define()
class _Config:
    N_STATIC_OBSTACLES: ROSParamT[tuple[int, int]]
    N_INTERACTIVE_OBSTACLES: ROSParamT[tuple[int, int]]
    N_DYNAMIC_OBSTACLES: ROSParamT[tuple[int, int]]

    MODELS_STATIC_OBSTACLES: ROSParamT[list[str]]
    MODELS_INTERACTIVE_OBSTACLES: ROSParamT[list[str]]
    MODELS_DYNAMIC_OBSTACLES: ROSParamT[list[str]]


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

    _config: _Config

    def reset(self, **kwargs) -> Obstacles:
        """
        Resets the obstacle generation with the specified parameters.

        Args:
            **kwargs: Additional keyword arguments for customizing the obstacle generation.
                N_STATIC_OBSTACLES (int): Number of static obstacles.
                N_INTERACTIVE_OBSTACLES (int): Number of interactive obstacles.
                N_DYNAMIC_OBSTACLES (int): Number of dynamic obstacles.
                MODELS_STATIC_OBSTACLES (dict[str, float]): dictionary of static obstacle models and their weights.
                MODELS_INTERACTIVE_OBSTACLES (dict[str, float]): dictionary of interactive obstacle models and their weights.
                MODELS_DYNAMIC_OBSTACLES (dict[str, float]): dictionary of dynamic obstacle models and their weights.

        Returns:
            tuple[list[Obstacle], list[DynamicObstacle]]: A tuple containing the generated obstacles and dynamic obstacles.

        """

        N_STATIC_OBSTACLES: int = kwargs.get(
            "N_STATIC_OBSTACLES",
            self.node.conf.General.RNG.value.integers(
                *self._config.N_STATIC_OBSTACLES.value,
                endpoint=True
            ),
        )
        N_INTERACTIVE_OBSTACLES: int = kwargs.get(
            "N_INTERACTIVE_OBSTACLES",
            self.node.conf.General.RNG.value.integers(
                *self._config.N_INTERACTIVE_OBSTACLES.value,
                endpoint=True
            ),
        )

        N_DYNAMIC_OBSTACLES: int = kwargs.get(
            "N_DYNAMIC_OBSTACLES",
            self.node.conf.General.RNG.value.integers(
                *self._config.N_DYNAMIC_OBSTACLES.value,
                endpoint=True
            ),
        )

        class ModelList(dict[str, float]):

            @classmethod
            def fromkeys(cls, *args, **kwargs) -> "ModelList":
                return cls(super().fromkeys(*args, **kwargs))

            @property
            def a(self) -> list[str]:
                return list(self.keys())

            @property
            def p(self) -> list[float]:
                _p = np.array(list(self.values()), dtype=float)
                _p /= _p.sum()
                return _p.tolist()

            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)

        MODELS_STATIC_OBSTACLES = ModelList.fromkeys(
            kwargs.get(
                "MODELS_STATIC_OBSTACLES",
                self._config.MODELS_STATIC_OBSTACLES.value
            ),
            1,
        )
        MODELS_INTERACTIVE_OBSTACLES = ModelList.fromkeys(
            kwargs.get(
                "MODELS_INTERACTIVE_OBSTACLES",
                self._config.MODELS_INTERACTIVE_OBSTACLES.value
            ),
            1,
        )

        MODELS_DYNAMIC_OBSTACLES = ModelList.fromkeys(
            kwargs.get(
                "MODELS_DYNAMIC_OBSTACLES",
                self._config.MODELS_DYNAMIC_OBSTACLES.value
            ),
            1,
        )

        # rospy.logwarn(f"{MODELS_DYNAMIC_OBSTACLES}")

        def indexer() -> Callable[..., int]:
            indices: dict[str, Iterator[int]] = dict()

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
            safe_dist=1
        )

        _positions = [
            PositionOrientation(
                x=pos.x,
                y=pos.y,
                orientation=2 * np.pi * self.node.conf.General.RNG.value.random(),
            )
            for pos in points[
                : (N_STATIC_OBSTACLES + N_INTERACTIVE_OBSTACLES + N_DYNAMIC_OBSTACLES)
            ]
        ]
        positions = iter(_positions)

        _waypoints = [
            PositionRadius(pos.x, pos.y, 1)
            for pos in points[
                (N_STATIC_OBSTACLES + N_INTERACTIVE_OBSTACLES + N_DYNAMIC_OBSTACLES):
            ]
        ]
        waypoints = iter(_waypoints)

        obstacles: list[Obstacle] = []

        # Create static obstacles
        if N_STATIC_OBSTACLES:
            index = indexer()
            obstacles += [
                ITF_Obstacle.create_obstacle(
                    self.node,
                    self._PROPS,
                    name=f"S_{model}_{index(model)}",
                    model=self._PROPS.model_loader.bind(model),
                    position=next(positions),
                )
                for model in self.node.conf.General.RNG.value.choice(
                    a=MODELS_STATIC_OBSTACLES.a,
                    p=MODELS_STATIC_OBSTACLES.p,
                    size=N_STATIC_OBSTACLES,
                )
            ]

        # Create interactive obstacles
        if N_INTERACTIVE_OBSTACLES:
            index = indexer()

            obstacles += [
                ITF_Obstacle.create_obstacle(
                    self.node,
                    self._PROPS,
                    name=f"I_{model}_{index(model)}",
                    model=self._PROPS.model_loader.bind(model),
                    position=next(positions),
                )
                for model in self.node.conf.General.RNG.value.choice(
                    a=MODELS_INTERACTIVE_OBSTACLES.a,
                    p=MODELS_INTERACTIVE_OBSTACLES.p,
                    size=N_INTERACTIVE_OBSTACLES,
                )
            ]

        # Create dynamic obstacles

        dynamic_obstacles: list[DynamicObstacle] = []

        if N_DYNAMIC_OBSTACLES:
            index = indexer()

            dynamic_obstacles += [
                ITF_Obstacle.create_dynamic_obstacle(
                    self.node,
                    self._PROPS,
                    name=f"D_{model}_{index(model)}",
                    model=self._PROPS.dynamic_model_loader.bind(model),
                    waypoints=list(
                        itertools.islice(
                            waypoints,
                            waypoints_per_ped)),
                    position=next(positions),
                )
                for model in self.node.conf.General.RNG.value.choice(
                    a=MODELS_DYNAMIC_OBSTACLES.a,
                    p=MODELS_DYNAMIC_OBSTACLES.p,
                    size=N_DYNAMIC_OBSTACLES,
                )
            ]

        return obstacles, dynamic_obstacles

    def __init__(self, **kwargs):
        TM_Obstacles.__init__(self, **kwargs)

        def param_to_tuple(v: typing.Any) -> tuple[int, int]:
            lo = int(v[0])
            hi = int(v[1] if len(v) >= 2 else v[0])
            lo, hi = min(lo, hi), max(lo, hi)
            return lo, hi

        def param_to_modellist(loader: ModelLoader,
                               v: typing.Any) -> list[str]:
            if len(v):
                return v
            return list(loader.models)

        STATIC = 'static'
        INTERACTIVE = 'interactive'
        DYNAMIC = 'dynamic'

        self._config = _Config(
            N_STATIC_OBSTACLES=self.node.ROSParam[tuple[int, int]](
                self.namespace(STATIC, 'n'),
                [5, 15],
                parse=param_to_tuple
            ),
            N_INTERACTIVE_OBSTACLES=self.node.ROSParam[tuple[int, int]](
                self.namespace(INTERACTIVE, 'n'),
                [0, 0],
                parse=param_to_tuple
            ),
            N_DYNAMIC_OBSTACLES=self.node.ROSParam[tuple[int, int]](
                self.namespace(DYNAMIC, 'n'),
                [1, 5],
                parse=param_to_tuple
            ),

            MODELS_STATIC_OBSTACLES=self.node.ROSParam[list[str]](
                self.namespace(STATIC, 'models'),
                [],
                type_=rclpy.Parameter.Type.STRING_ARRAY,
                parse=functools.partial(
                    param_to_modellist,
                    self._PROPS.model_loader
                )
            ),
            MODELS_INTERACTIVE_OBSTACLES=self.node.ROSParam[list[str]](
                self.namespace(INTERACTIVE, 'models'),
                [],
                type_=rclpy.Parameter.Type.STRING_ARRAY,
                parse=functools.partial(
                    param_to_modellist,
                    self._PROPS.model_loader
                )
            ),
            MODELS_DYNAMIC_OBSTACLES=self.node.ROSParam[list[str]](
                self.namespace(DYNAMIC, 'models'),
                [],
                type_=rclpy.Parameter.Type.STRING_ARRAY,
                parse=functools.partial(
                    param_to_modellist,
                    self._PROPS.dynamic_model_loader
                )
            ),
        )
