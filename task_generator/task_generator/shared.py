from __future__ import annotations
import collections

import dataclasses
import os
from typing import (
    Callable,
    Collection,
    Dict,
    Iterable,
    List,
    Optional,
    Tuple,
    Type,
    TypeVar,
    overload,
)

import enum
import yaml
import rospy

T = TypeVar("T")
_unspecified = rospy.client._Unspecified()


def rosparam_get(cast: Type[T], param_name: str, default=_unspecified, strict: bool = False) -> T:
    val = rospy.get_param(param_name=param_name, default=default)

    if strict:
        if not isinstance(val, cast):
            raise ValueError(
                f"param {param_name} is not of type {cast} but {type(val)} with val {val}")
    else:
        try:
            val = cast(val)
        except ValueError as e:
            raise ValueError(f"could not cast {val} to {cast}", e)

    return val


class Namespace(str):
    def __call__(self, *args: str) -> Namespace:
        return Namespace(os.path.join(self, *args))

    @property
    def simulation_ns(self) -> Namespace:
        return Namespace(os.path.dirname(self))

    @property
    def robot_ns(self) -> Namespace:
        return Namespace(os.path.basename(os.path.normpath(self)))

    def remove_double_slash(self) -> Namespace:
        return Namespace(self.replace("//", "/"))


yaml.add_representer(Namespace, lambda dumper, data: dumper.represent_str(str(data)))


# TODO deprecate this in favor of Model.EMPTY
EMPTY_LOADER = lambda *_, **__: Model(
    type=ModelType.UNKNOWN, name="", description="", path=""
)


class ModelType(enum.Enum):
    UNKNOWN = ""
    URDF = "urdf"
    SDF = "sdf"
    YAML = "yaml"


@dataclasses.dataclass(frozen=True)
class Model:
    type: ModelType
    name: str
    description: str
    path: str

    @property
    def mapper(self) -> Callable[[Model], Model]:
        """
        Returns a (Model)->Model mapper that simply returns this model
        """
        return lambda m: self

    def replace(self, **kwargs) -> Model:
        """
        Wrapper for dataclasses.replace
        **kwargs: properties to replace
        """
        return dataclasses.replace(self, **kwargs)


Position = collections.namedtuple(
    "Position",
    ("x", "y")
)

PositionOrientation = collections.namedtuple(
    "PositionOrientation",
    ("x", "y", "orientation")
)

PositionRadius = collections.namedtuple(
    "PositionRadius",
    ("x", "y", "radius")
)


class ModelWrapper:
    _get: Callable[[Collection[ModelType]], Model]
    _name: str
    _override: Dict[ModelType, Tuple[bool, Callable[..., Model]]]

    def __init__(self, name: str):
        """
        Create new ModelWrapper
        @name: Name of the ModelWrapper (should match the underlying Models)
        """
        self._name = name
        self._override = dict()

    def clone(self) -> ModelWrapper:
        """
        Clone (shallow copy) this ModelWrapper instance
        """
        clone = ModelWrapper(self.name)
        clone._get = self._get
        clone._override = self._override
        return clone

    def override(
        self,
        model_type: ModelType,
        override: Callable[[Model], Model],
        noload: bool = False,
        name: Optional[str] = None,
    ) -> ModelWrapper:
        """
        Create new ModelWrapper with an overridden ModelType callback
        @model_type: Which ModelType to override
        @override: Mapping function (Model)->Model which replaces the loaded model with a new one
        @noload: (default: False) If True, indicates that the original Model is not used by the override function and a dummy Model can be passed to it instead
        @name: (optional) If set, overrides name of ModelWrapper
        """
        clone = self.clone()
        clone._override = {**self._override, model_type: (noload, override)}

        if name is not None:
            clone._name = name

        return clone

    @overload
    def get(self, only: ModelType, **kwargs) -> Model:
        ...

    """
        load specific model
        @only: single accepted ModelType
    """

    @overload
    def get(self, only: Collection[ModelType], **kwargs) -> Model:
        ...

    """
        load specific model from collection
        @only: collection of acceptable ModelTypes
    """

    @overload
    def get(self, **kwargs) -> Model:
        ...

    """
        load any available model
    """

    def get(
        self, only: ModelType | Collection[ModelType] | None = None, **kwargs
    ) -> Model:
        if only is None:
            only = self._override.keys()

        if isinstance(only, ModelType):
            return self.get([only])

        for model_type in only:
            if model_type in self._override:
                noload, mapper = self._override[model_type]

                if noload == True:
                    return mapper(EMPTY_LOADER())

                return mapper(self._get([model_type], **kwargs), **kwargs)

        return self._get(only, **kwargs)

    @property
    def name(self) -> str:
        """
        get name
        """
        return self._name

    @staticmethod
    def bind(
        name: str, callback: Callable[[Collection[ModelType]], Model]
    ) -> ModelWrapper:
        """
        Create new ModelWrapper with bound callback method
        """
        wrap = ModelWrapper(name)
        wrap._get = callback
        return wrap

    @staticmethod
    def Constant(name: str, models: Dict[ModelType, Model]) -> ModelWrapper:
        """
        Create new ModelWrapper from a dict of already existing models
        @name: name of model
        @models: dictionary of ModelType->Model mappings
        """

        def get(only: Collection[ModelType]) -> Model:
            if not len(only):
                only = list(models.keys())

            for model_type in only:
                if model_type in models:
                    return models[model_type]
            else:
                raise LookupError(
                    f"no matching model found for {name} (available: {list(models.keys())}, requested: {list(only)})"
                )

        return ModelWrapper.bind(name, get)

    @staticmethod
    def from_model(model: Model) -> ModelWrapper:
        """
        Create new ModelWrapper containing a single existing Model
        @model: Model to wrap
        """
        return ModelWrapper.Constant(name=model.name, models={model.type: model})

    @staticmethod
    def EMPTY() -> ModelWrapper:
        wrapper = ModelWrapper("__EMPTY")
        wrapper._get = EMPTY_LOADER
        return wrapper


@dataclasses.dataclass(frozen=True)
class EntityProps:
    position: PositionOrientation
    name: str
    model: ModelWrapper
    extra: Dict


@dataclasses.dataclass(frozen=True)
class ObstacleProps(EntityProps):
    ...


@dataclasses.dataclass(frozen=True)
class DynamicObstacleProps(ObstacleProps):
    waypoints: List[PositionRadius]


@dataclasses.dataclass(frozen=True)
class RobotProps(EntityProps):
    inter_planner: str
    local_planner: str
    agent: str
    record_data: bool


@dataclasses.dataclass(frozen=True)
class Obstacle(ObstacleProps):
    @staticmethod
    def parse(obj: Dict, model: ModelWrapper) -> "Obstacle":
        name = str(obj.get("name", ""))
        position = PositionOrientation(*obj.get("pos", (0, 0, 0)))

        return Obstacle(
            name=name,
            position=position,
            model=model,
            extra=obj,
        )


@dataclasses.dataclass(frozen=True)
class DynamicObstacle(DynamicObstacleProps):
    waypoints: Iterable[PositionRadius]

    @staticmethod
    def parse(obj: Dict, model: ModelWrapper) -> "DynamicObstacle":
        name = str(obj.get("name", ""))
        position = PositionOrientation(*obj.get("pos", (0, 0, 0)))
        waypoints = [PositionRadius(*waypoint)
                     for waypoint in obj.get("waypoints", [])]

        return DynamicObstacle(
            name=name, position=position, model=model, waypoints=waypoints, extra=obj
        )


@dataclasses.dataclass(frozen=True)
class WallObstacle:
    name: str
    start: Position
    end: Position


def _gen_init_pos(steps: int, x: int = 1, y: int = 0):
    steps = max(steps, 1)

    while True:
        yield PositionOrientation(1, 1, 0)

    while True:
        x += y == steps
        y %= steps
        yield PositionOrientation(-x, y, 0)
        y += 1


gen_init_pos = _gen_init_pos(10)


@dataclasses.dataclass(frozen=True)
class Robot(RobotProps):
    @staticmethod
    def parse(obj: Dict, model: ModelWrapper) -> "Robot":
        name = str(obj.get("name", ""))
        position = PositionOrientation(*obj.get("pos", next(gen_init_pos)))
        inter_planner = str(obj.get("inter", ""))
        local_planner = str(obj.get("planner", ""))
        agent = str(obj.get("agent", ""))
        record_data = bool(obj.get("record_data", rosparam_get(bool, "record_data", False)))

        return Robot(
            name=name,
            position=position,
            inter_planner=inter_planner,
            local_planner=local_planner,
            model=model,
            agent=agent,
            record_data=record_data,
            extra=obj,
        )
