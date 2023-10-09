from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Collection, Dict, Iterable, List, Optional, Tuple, overload
from geometry_msgs.msg import Pose, Quaternion
import enum

EMPTY_LOADER = lambda *_, **__:Model(type=ModelType.UNKNOWN, name="", description="")

class ModelType(enum.Enum):
    UNKNOWN = ""
    URDF = "urdf"
    SDF = "sdf"
    YAML = "yaml"


@dataclass(frozen=True)
class Model:
    type: ModelType
    name: str
    description: str

ForbiddenZone = Tuple[float, float, float]

PositionOrientation = Tuple[float, float, float]
Waypoint = Tuple[float, float, float]
Position = Tuple[float, float]



class ModelWrapper:

    get: Callable[[Collection[ModelType]], Model]
    _name: str

    @staticmethod
    def bind(name: str, callback: Callable[..., Model]) -> "ModelWrapper":
        wrap = ModelWrapper()
        wrap._name = name
        wrap.get = callback
        return wrap

    @staticmethod
    def Constant(name: str, models: Dict[ModelType, Model]) -> ModelWrapper:

        wrap = ModelWrapper()

        def get(only: Collection[ModelType]) -> Model:
            if only is None:
                only = list(models.keys())

            for model_type in only:
                if model_type in models:
                    return models[model_type]
            else:
                raise LookupError(f"no matching model found for {name} (available: {list(models.keys())}, requested: {list(only)})")

        wrap.get = get

        return wrap
    
    @staticmethod
    def from_model(model: Model) -> ModelWrapper:
        return ModelWrapper.Constant(name=model.name, models={model.type: model})


@dataclass
class HasModel:
    model: ModelWrapper


@dataclass
class ObstacleProps(HasModel):
    position: PositionOrientation
    name: str
    extra: Dict

@dataclass
class DynamicObstacleProps(ObstacleProps):
    waypoints: List[Waypoint]

@dataclass
class RobotProps(ObstacleProps):
    planner: str
    namespace: str
    agent: str
    record_data: bool


@dataclass
class Obstacle(ObstacleProps):
    @staticmethod
    def parse(obj: Dict, **kwargs) -> "Obstacle":
        #TODO
        ...

@dataclass
class DynamicObstacle(DynamicObstacleProps):
    waypoints: Iterable[Waypoint]

    @staticmethod
    def parse(obj: Dict, **kwargs) -> "DynamicObstacle":
        #TODO
        ...

@dataclass
class Robot(RobotProps):
    @staticmethod
    def parse(obj: Dict, **kwargs) -> "Robot":
        #TODO
        ...


@dataclass
class ScenarioObstacles:
    dynamic: List[DynamicObstacle]
    static: List[Obstacle]
    interactive: List[Obstacle]


ScenarioMap = str


@dataclass
class RobotGoal:
    start: PositionOrientation
    goal: PositionOrientation


@dataclass
class Scenario:
    obstacles: ScenarioObstacles
    map: ScenarioMap
    resets: int
    robots: List[RobotGoal]