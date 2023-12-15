import itertools
import math
import random
from typing import Callable, Dict, Iterator, List
from task_generator.constants import Constants
from task_generator.shared import DynamicObstacle, Obstacle, PositionOrientation, PositionRadius
from task_generator.tasks.obstacles import Obstacles, TM_Obstacles
from task_generator.tasks.obstacles.utils import ITF_Obstacle
from task_generator.tasks.task_factory import TaskFactory

@TaskFactory.register_obstacles(Constants.TaskMode.TM_Obstacles.RANDOM)
class TM_Random(TM_Obstacles):
    def reset(self, **kwargs) -> Obstacles:

        N_STATIC_OBSTACLES: int = kwargs.get("N_STATIC_OBSTACLES", 1)
        N_INTERACTIVE_OBSTACLES: int = kwargs.get("N_INTERACTIVE_OBSTACLES", 1)
        N_DYNAMIC_OBSTACLES: int = kwargs.get("N_DYNAMIC_OBSTACLES", 1)

        MODELS_STATIC_OBSTACLES: Dict[str, float] = dict.fromkeys(kwargs.get("MODELS_STATIC_OBSTACLES", ["shelf"]) or self._PROPS.model_loader.models, 1)
        MODELS_INTERACTIVE_OBSTACLES: Dict[str, float] = dict.fromkeys(kwargs.get("MODELS_INTERACTIVE_OBSTACLES", ["shelf"]) or self._PROPS.model_loader.models, 1)
        MODELS_DYNAMIC_OBSTACLES: Dict[str, float] = dict.fromkeys(kwargs.get("MODELS_DYNAMIC_OBSTACLES", ["actor1"]) or self._PROPS.dynamic_model_loader.models, 1)

        def indexer() -> Callable[..., int]:
            indices: Dict[str, Iterator[int]] = dict()

            def index(model: str):
                if model not in indices:
                    indices[model] = itertools.count(1)
                return next(indices[model])
            return index

        waypoints_per_ped = 2
        points = self._PROPS.world_manager.get_positions_on_map(n=N_STATIC_OBSTACLES + N_INTERACTIVE_OBSTACLES + N_DYNAMIC_OBSTACLES*(1+waypoints_per_ped), safe_dist=1)

        _positions = [PositionOrientation(*pos, 2*math.pi * random.random()) for pos in points[:(N_STATIC_OBSTACLES + N_INTERACTIVE_OBSTACLES + N_DYNAMIC_OBSTACLES)]]
        positions = iter(_positions)

        _waypoints = [PositionRadius(*pos, 1) for pos in points[(N_STATIC_OBSTACLES + N_INTERACTIVE_OBSTACLES + N_DYNAMIC_OBSTACLES):]]
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
                    position=next(positions)
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
                    position=next(positions)
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