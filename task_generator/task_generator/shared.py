from __future__ import annotations

import enum
import os
import typing
from typing import (Callable, Collection, Optional, Type, TypeVar, overload)

import attrs
import geometry_msgs.msg as geometry_msgs
import rclpy
import rclpy.node
import yaml

from task_generator.utils.geometry import (euler_from_quaternion,
                                           quaternion_from_euler)

_node: rclpy.node.Node


def configure_node(node: rclpy.node.Node):
    global _node
    _node = node


T = TypeVar("T")


def rosparam_get(
    cast: Type[T], param_name: str, default: T
) -> T:
    """
    # TODO deprecate in favor of ROSParamServer.rosparam[T].get
    Get typed ros parameter (strict)
    @cast: Return type of function
    @param_name: Name of parameter on parameter server
    @default: Default value. Raise ValueError is default is unset and parameter can't be found.
    """
    return _node.rosparam[cast].get(param_name, default)


def rosparam_set(
    param_name: str, value: typing.Any
) -> bool:
    """
    # TODO deprecate in favor of ROSParamServer.rosparam[T].set
    """
    return _node.rosparam.set(param_name, value)


class Namespace(str):
    def __call__(self, *args: str) -> Namespace:
        return Namespace(os.path.join(self, *args)).remove_double_slash()

    def ParamNamespace(self) -> ParamNamespace:
        return ParamNamespace('')(*self.split('/'))

    @property
    def simulation_ns(self) -> Namespace:
        if len(self.split("/")) < 3:
            return self
        return Namespace(os.path.dirname(self))

    @property
    def robot_ns(self) -> Namespace:
        return Namespace(os.path.basename(os.path.normpath(self)))

    def remove_double_slash(self) -> Namespace:
        return Namespace(self.replace("//", "/"))


class ParamNamespace(Namespace):
    def __call__(self, *args: str) -> ParamNamespace:
        return ParamNamespace(
            '.'.join((
                *((self,) if self else []),
                *args)
            )
        )

    def SlashNamespace(self) -> Namespace:
        return Namespace('')(*self.split('.'))


yaml.add_representer(
    Namespace,
    lambda dumper,
    data: dumper.represent_str(
        str(data)))


# TODO deprecate this in favor of Model.EMPTY
EMPTY_LOADER = lambda *_, **__: Model(
    type=ModelType.UNKNOWN, name="", description="", path=""
)


class ModelType(enum.Enum):
    UNKNOWN = ""
    URDF = "urdf"
    SDF = "sdf"
    YAML = "yaml"
    USD = "usd"


@attrs.frozen()
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
        Wrapper for attrs.evolve
        **kwargs: properties to replace
        """
        return attrs.evolve(self, **kwargs)


@attrs.frozen()
class Position:
    """
    2D position
    """
    x: float = attrs.field(converter=float)
    y: float = attrs.field(converter=float)


@attrs.frozen()
class PositionOrientation(Position):
    """
    2D position with 2D yaw
    """
    orientation: float = attrs.field(converter=float)

    @classmethod
    def from_pose(
        cls,
        pose: geometry_msgs.Pose
    ) -> "PositionOrientation":
        """
        parse geometry_msgs.msg.Pose
        """
        return cls(
            x=pose.position.x,
            y=pose.position.y,
            orientation=euler_from_quaternion(
                x=pose.orientation.x,
                y=pose.orientation.y,
                z=pose.orientation.z,
                w=pose.orientation.w
            )[2]
        )

    def to_pose(self) -> geometry_msgs.Pose:
        """
        return self as geometry_msgs.msg.Pose
        """

        quat = quaternion_from_euler(
            0.0,
            0.0,
            self.orientation,
            axes="xyzs"
        )

        pose = geometry_msgs.Pose(
            position=geometry_msgs.Point(
                x=self.x,
                y=self.y,
            ),
            orientation=geometry_msgs.Quaternion(
                x=quat[0],
                y=quat[1],
                z=quat[2],
                w=quat[3],
            )
        )

        return pose


@attrs.frozen()
class PositionRadius(Position):
    """
    2D position with 2D yaw
    """
    radius: float = attrs.field(converter=lambda x: max(0., float(x)))

    @classmethod
    def parse(cls, v: typing.Iterable[float] | PositionRadius) -> "PositionRadius":
        if isinstance(v, PositionRadius):
            return v
        return cls(*v)


class ModelWrapper:
    _get: Callable[[Collection[ModelType], dict], Model]
    _name: str
    _override: dict[ModelType, tuple[bool, Callable[..., Model]]]

    def __init__(
        self,
        name: str,
        callback: Callable[[Collection[ModelType], dict], Model] | None = None
    ):
        """
        Create new ModelWrapper
        @name: Name of the ModelWrapper (should match the underlying Models)
        """
        if callback is None:
            callback = EMPTY_LOADER
        self._name = name
        self._get = callback
        self._override = dict()

    def clone(self) -> ModelWrapper:
        """
        Clone (shallow copy) this ModelWrapper instance
        """
        clone = ModelWrapper(self.name, self._get)
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
    def get(
        self,
        only: ModelType,
        *,
        loader_args: dict | None = None,
        **kwargs) -> Model: ...
    """
        load specific model
        @only: single accepted ModelType
    """

    @overload
    def get(
        self,
        only: Collection[ModelType],
        *,
        loader_args: dict | None = None,
        **kwargs
    ) -> Model: ...
    """
        load specific model from collection
        @only: collection of acceptable ModelTypes
    """

    @overload
    def get(
        self,
        *,
        loader_args: dict | None = None,
        **kwargs
    ) -> Model: ...
    """
        load any available model
    """

    def get(
        self,
        only: ModelType | Collection[ModelType] | None = None,
        *,
        loader_args: dict | None = None,
        **kwargs,
    ) -> Model:
        if only is None:
            only = self._override.keys()

        if isinstance(only, ModelType):
            return self.get([only])

        if loader_args is None:
            loader_args = {}

        for model_type in only:
            if model_type in self._override:
                noload, mapper = self._override[model_type]

                if noload == True:
                    return mapper(EMPTY_LOADER())

                return mapper(self._get([model_type], loader_args), **kwargs)

        return self._get(only, loader_args)

    @property
    def name(self) -> str:
        """
        get name
        """
        return self._name

    @staticmethod
    def Constant(name: str, models: dict[ModelType, Model]) -> ModelWrapper:
        """
        Create new ModelWrapper from a dict of already existing models
        @name: name of model
        @models: dictionary of ModelType->Model mappings
        """

        def get(only: Collection[ModelType], loader_args: dict) -> Model:
            if not len(only):
                only = list(models.keys())

            for model_type in only:
                if model_type in models:
                    return models[model_type]
            else:
                raise LookupError(
                    f"no matching model found for {name} (available: {list(models.keys())}, requested: {list(only)})"
                )

        return ModelWrapper(name, get)

    @staticmethod
    def from_model(model: Model) -> ModelWrapper:
        """
        Create new ModelWrapper containing a single existing Model
        @model: Model to wrap
        """
        return ModelWrapper.Constant(
            name=model.name,
            models={model.type: model}
        )

    @staticmethod
    def EMPTY() -> ModelWrapper:
        wrapper = ModelWrapper("__EMPTY", EMPTY_LOADER)
        return wrapper


@attrs.frozen()
class Wall:
    Start: Position
    End: Position
    height: float = attrs.field(converter=float, default=2.)
    texture_material: str = ''  # not implemented

    @classmethod
    def parse(cls, obj: list) -> "Wall":
        kwargs = {}
        if len(obj) > 2 and isinstance(obj[2], dict):
            kwargs = obj[2]
        return cls(
            **kwargs,
            Start=Position(x=obj[0][0], y=obj[0][1]),
            End=Position(x=obj[1][0], y=obj[1][1]),
        )


@attrs.frozen()
class Entity:
    position: PositionOrientation
    name: str
    model: ModelWrapper
    extra: dict = attrs.field(factory=dict, kw_only=True)

    def asdict(self, expand_extra: bool = True) -> dict:
        if expand_extra:
            return {
                **attrs.asdict(self, filter=lambda a, v: a.name != 'extra'),
                **self.extra,
            }
        return attrs.asdict(self)


@attrs.frozen()
class Obstacle(Entity):
    @classmethod
    def parse(cls, obj: dict, model: ModelWrapper) -> "Obstacle":
        name = str(obj.get("name", ""))
        position = PositionOrientation(*obj.get("pos", (0, 0, 0)))

        return Obstacle(
            name=name,
            position=position,
            model=model,
            extra=obj,
        )


@attrs.frozen()
class DynamicObstacle(Obstacle):
    waypoints: list[PositionRadius]

    @classmethod
    def parse(cls, obj: dict, model: ModelWrapper) -> "DynamicObstacle":

        base = Obstacle.parse(obj, model)
        waypoints = [PositionRadius.parse(waypoint) for waypoint in obj.get("waypoints", [])]

        return DynamicObstacle(
            **attrs.asdict(base, recurse=False),
            waypoints=waypoints,
        )


@attrs.frozen()
class Robot(Entity):
    inter_planner: str
    local_planner: str
    global_planner: str
    agent: str
    record_data_dir: Optional[str] = None

    def compatible(self, value: Robot) -> bool:
        return self.model.name == value.model.name \
            and self.local_planner == value.local_planner \
            and self.global_planner == value.global_planner \
            and self.agent == value.agent \


    def __eq__(self, value: object) -> bool:
        if not isinstance(value, Robot):
            return False

        return self.compatible(value) \
            and self.name == value.name \
            and self.record_data_dir == value.record_data_dir

    @property
    def frame(self) -> str:
        if not self.name:
            return ''
        return self.name + '/'

    @classmethod
    def parse(cls, obj: dict, model: ModelWrapper) -> "Robot":
        name = str(obj.get("name", ""))
        position = PositionOrientation(*obj.get("pos", (0, 0, 0)))
        inter_planner = str(
            obj.get("inter_planner", rosparam_get(str, "inter_planner", ""))
        )
        local_planner = str(
            obj.get("local_planner", rosparam_get(str, "local_planner", ""))
        )
        global_planner = str(
            obj.get("global_planner", rosparam_get(str, "global_planner", ""))
        )
        agent = str(obj.get("agent", rosparam_get(str, "agent_name", "")))
        record_data = obj.get(
            "record_data_dir", rosparam_get(str, "record_data_dir", None)
        )

        return Robot(
            name=name,
            position=position,
            inter_planner=inter_planner,
            local_planner=local_planner,
            global_planner=global_planner,
            model=model,
            agent=agent,
            record_data_dir=record_data,
            extra=obj,
        )


def DefaultParameter(value: typing.Any) -> rclpy.Parameter | None:
    if value is None:
        return None
    # if isinstance(value, rclpy.Parameter.Type):
    #     return None
    return rclpy.Parameter(
        '',
        value=value,
    )
