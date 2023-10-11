from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Collection, Dict, Iterable, List, Sequence, Tuple

import enum


# TODO maybe move this to utils/model

EMPTY_LOADER = lambda *_, **__: Model(type=ModelType.UNKNOWN,
                                      name="",
                                      description="",
                                      path="")


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
    path: str


ForbiddenZone = Tuple[float, float, float]

PositionOrientation = Tuple[float, float, float]
Waypoint = PositionOrientation
Position = Tuple[float, float]


class ModelWrapper:

    _get: Callable[[Collection[ModelType]], Model]
    _name: str
    _override: Dict[ModelType, Model]

    def __init__(self, name: str):
        self._name = name
        self._override = dict()

    def clone(self) -> ModelWrapper:
        clone = ModelWrapper(self.name)
        clone._get = self._get
        clone._override = self._override
        return clone

    def override(self, model_type: ModelType, model: Model) -> ModelWrapper:
        clone = self.clone()
        clone._override = {**self._override, model_type: model}
        return clone

    def get(self, only: Collection[ModelType]) -> Model:
        for model_type in (only or self._override.keys()):
            if model_type in self._override:
                return self._override[model_type]

        return self._get(only)

    @property
    def name(self) -> str:
        return self._name

    @staticmethod
    def bind(name: str, callback: Callable[..., Model]) -> "ModelWrapper":
        wrap = ModelWrapper(name)
        wrap._get = callback
        return wrap

    @staticmethod
    def Constant(name: str, models: Dict[ModelType, Model]) -> ModelWrapper:

        wrap = ModelWrapper(name)

        def get(only: Collection[ModelType]) -> Model:
            if not len(only):
                only = list(models.keys())

            for model_type in only:
                if model_type in models:
                    return models[model_type]
            else:
                raise LookupError(
                    f"no matching model found for {name} (available: {list(models.keys())}, requested: {list(only)})")

        wrap._get = get

        return wrap

    @staticmethod
    def from_model(model: Model) -> ModelWrapper:
        return ModelWrapper.Constant(name=model.name, models={model.type: model})

@dataclass(frozen=True)
class HasModel:
    model: ModelWrapper

@dataclass(frozen=True)
class ObstacleProps(HasModel):
    position: PositionOrientation
    name: str
    extra: Dict


@dataclass(frozen=True)
class DynamicObstacleProps(ObstacleProps):
    waypoints: List[Waypoint]


@dataclass(frozen=True)
class RobotProps(ObstacleProps):
    planner: str
    namespace: str
    agent: str
    record_data: bool

def parse_Point3D(obj: Sequence, fill: float = 0.) -> Tuple[float, float, float]:
    position: Tuple[float, ...] = tuple([float(v) for v in obj[:3]])

    if len(position) < 3:
        position = (*position, *((3-len(position)) * [fill]))

    return (position[0], position[1], position[2])

@dataclass(frozen=True)
class Obstacle(ObstacleProps):
    @staticmethod
    def parse(obj: Dict, model: ModelWrapper) -> "Obstacle":

        name = str(obj.get("name", "MISSING"))
        position = parse_Point3D(obj.get("pos", (0, 0, 0)))

        return Obstacle(
            name=name,
            position=position,
            model=model,
            extra=obj
        )

@dataclass(frozen=True)
class DynamicObstacle(DynamicObstacleProps):
    waypoints: Iterable[Waypoint]

    @staticmethod
    def parse(obj: Dict, model: ModelWrapper) -> "DynamicObstacle":

        name = str(obj.get("name", "MISSING"))
        position = parse_Point3D(obj.get("pos", (0, 0, 0)))
        waypoints = [parse_Point3D(waypoint) for waypoint in obj.get("waypoints", [])]

        return DynamicObstacle(
            name=name,
            position=position,
            model=model,
            waypoints=waypoints,
            extra=obj
        )

@dataclass(frozen=True)
class Robot(RobotProps):
    @staticmethod
    def parse(obj: Dict, **kwargs) -> "Robot":
        # TODO
        raise NotImplementedError()
