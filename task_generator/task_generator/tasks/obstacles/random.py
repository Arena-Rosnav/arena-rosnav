import dataclasses
import functools
import itertools
import typing
from typing import Callable, Dict, Iterator, List

import numpy as np

from task_generator.shared import (DynamicObstacle, Obstacle,
                                   PositionOrientation, PositionRadius)
from task_generator.tasks.obstacles import Obstacles, TM_Obstacles
from task_generator.tasks.obstacles.utils import ITF_Obstacle
from task_generator.utils import ModelLoader
from task_generator.utils.ros_params import ROSParam


@dataclasses.dataclass
class _Config:
    N_STATIC_OBSTACLES: ROSParam[typing.Tuple[int, int]]
    N_INTERACTIVE_OBSTACLES: ROSParam[typing.Tuple[int, int]]
    N_DYNAMIC_OBSTACLES: ROSParam[typing.Tuple[int, int]]

    MODELS_STATIC_OBSTACLES: ROSParam[List[str]]
    MODELS_INTERACTIVE_OBSTACLES: ROSParam[List[str]]
    MODELS_DYNAMIC_OBSTACLES: ROSParam[List[str]]


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
                MODELS_STATIC_OBSTACLES (Dict[str, float]): Dictionary of static obstacle models and their weights.
                MODELS_INTERACTIVE_OBSTACLES (Dict[str, float]): Dictionary of interactive obstacle models and their weights.
                MODELS_DYNAMIC_OBSTACLES (Dict[str, float]): Dictionary of dynamic obstacle models and their weights.

        Returns:
            Tuple[List[Obstacle], List[DynamicObstacle]]: A tuple containing the generated obstacles and dynamic obstacles.

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
            def a(self) -> typing.List[str]:
                return list(self.keys())

            @property
            def p(self) -> typing.List[float]:
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

        obstacles: List[Obstacle] = []

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

        dynamic_obstacles: List[DynamicObstacle] = []

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

        def param_to_tuple(v: typing.Any) -> typing.Tuple[int, int]:
            lo = int(v[0])
            hi = int(v[1] if len(v) >= 2 else v[0])
            lo, hi = min(lo, hi), max(lo, hi)
            return lo, hi

        def param_to_modellist(loader: ModelLoader,
                               v: typing.Any) -> typing.List[str]:
            if len(v):
                return v.split(';')
            return list(loader.models)

        self._config = _Config(
            N_STATIC_OBSTACLES=self.node.ROSParam[typing.Tuple[int, int]](
                "N_STATIC_OBSTACLES", [5, 15], parse=param_to_tuple),
            N_INTERACTIVE_OBSTACLES=self.node.ROSParam[typing.Tuple[int, int]](
                "N_INTERACTIVE_OBSTACLES", [0, 0], parse=param_to_tuple),
            N_DYNAMIC_OBSTACLES=self.node.ROSParam[typing.Tuple[int, int]](
                "N_DYNAMIC_OBSTACLES", [1, 5], parse=param_to_tuple),

            MODELS_STATIC_OBSTACLES=self.node.ROSParam[List[str]](
                'MODELS_STATIC_OBSTACLES', '', parse=functools.partial(param_to_modellist, self._PROPS.model_loader)),
            MODELS_INTERACTIVE_OBSTACLES=self.node.ROSParam[List[str]](
                'MODELS_INTERACTIVE_OBSTACLES', '', parse=functools.partial(param_to_modellist, self._PROPS.model_loader)),
            MODELS_DYNAMIC_OBSTACLES=self.node.ROSParam[List[str]](
                'MODELS_DYNAMIC_OBSTACLES', 'ugly', parse=functools.partial(param_to_modellist, self._PROPS.dynamic_model_loader)),
        )
