from __future__ import annotations
from ament_index_python.packages import get_package_share_directory
import yaml
import enum
import collections
import typing
from geometry_msgs.msg import Pose, Twist, Point
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
    Union,
    overload,
)
import rclpy
import rclpy.node

_node: rclpy.node.Node


def configure_node(node: rclpy.node.Node):
    global _node
    _node = node


T = TypeVar("T")


def rosparam_get(
    cast: Type[T], param_name: str, default: typing.Optional[T]
) -> T:
    """
    Get typed ros parameter (strict)
    @cast: Return type of function
    @param_name: Name of parameter on parameter server
    @default: Default value. Raise ValueError is default is unset and parameter can't be found.
    """
    # if "task_generator_node" not in  get_nodes():
    global _node
    return _node.get_parameter_or(param_name, DefaultParameter(default)).value

    if val == _notfound:
        if isinstance(default, _UNSPECIFIED):
            raise ValueError(f"required parameter {param_name} is not set")
        return default

    try:
        return cast(val)
    except ValueError as e:
        raise ValueError(
            f"could not cast {val} to {cast} of param {param_name}") from e


def rosparam_set(
    param_name: str, value: typing.Any
) -> bool:
    global _node
    return _node.set_parameters(
        [rclpy.parameter.Parameter(param_name, value=value)])[0].successful


class Namespace(str):
    def __call__(self, *args: str) -> Namespace:
        return Namespace(os.path.join(self, *args)).remove_double_slash()

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


Position = collections.namedtuple("Position", ("x", "y"))

PositionOrientation = collections.namedtuple(
    "PositionOrientation", ("x", "y", "orientation")
)

PositionRadius = collections.namedtuple("PositionRadius", ("x", "y", "radius"))


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
    def get(self, only: ModelType, **kwargs) -> Model: ...
    """
        load specific model
        @only: single accepted ModelType
    """

    @overload
    def get(self, only: Collection[ModelType], **kwargs) -> Model: ...
    """
        load specific model from collection
        @only: collection of acceptable ModelTypes
    """

    @overload
    def get(self, **kwargs) -> Model: ...
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
        return ModelWrapper.Constant(
            name=model.name, models={model.type: model})

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
    record_data_dir: Optional[str] = None


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


def load_config(filename: str = "default.yaml") -> dict:
    """Load config from YAML file in arena_bringup configs."""
    # first priority: Source space
    source_path = os.path.join(
        os.environ.get('HOME', ''),
        "arena4_ws/src/arena/arena-rosnav/arena_bringup/configs/hunav_agents",
        filename
    )

    # second priority: Install space
    install_path = os.path.join(
        get_package_share_directory("arena_bringup"),
        "configs",
        "hunav_agents",
        filename
    )

    print(f"\n========== TRYING CONFIG PATHS ==========")
    print(f"Source path: {source_path}")
    print(f"Install path: {install_path}")

    # try first source, then install path
    if os.path.exists(source_path):
        config_path = source_path
        print(f"\nUsing source space config: {config_path}")
    elif os.path.exists(install_path):
        config_path = install_path
        print(f"\nUsing install space config: {config_path}")
    else:
        raise FileNotFoundError(
            f"Config file not found in either:\n- {source_path}\n- {install_path}")

    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            params = config['hunav_loader']['ros__parameters']

            print("\n========== FULL PARAMETER OVERVIEW ==========")
            print("\n=== Available Agents ===")
            print(f"Agents: {params['agents']}")

            print("\n=== Individual Agent Configurations ===")
            for agent in params['agents']:
                agent_config = params[agent]
                print(f"\nAgent: {agent}")
                print("Basic Properties:")
                print(f"- ID: {agent_config.get('id')}")
                print(f"- Skin: {agent_config.get('skin')}")
                print(f"- Group ID: {agent_config.get('group_id')}")
                print(f"- Max Velocity: {agent_config.get('max_vel')}")
                print(f"- Radius: {agent_config.get('radius')}")

                print("\nBehavior:")
                behavior = agent_config.get('behavior', {})
                print(
                    f"- Type: {behavior.get('type')}  # REGULAR=1, IMPASSIVE=2, SURPRISED=3, SCARED=4, CURIOUS=5, THREATENING=6")
                print(
                    f"- Configuration: {behavior.get('configuration')}  # default=0, custom=1, random_normal=2, random_uniform=3")
                print(f"- Duration: {behavior.get('duration')}")
                print(f"- Once: {behavior.get('once')}")
                print(f"- Velocity: {behavior.get('vel')}")
                print(f"- Distance: {behavior.get('dist')}")
                print(
                    f"- Goal Force Factor: {behavior.get('goal_force_factor')}")
                print(
                    f"- Obstacle Force Factor: {behavior.get('obstacle_force_factor')}")
                print(
                    f"- Social Force Factor: {behavior.get('social_force_factor')}")
                print(
                    f"- Other Force Factor: {behavior.get('other_force_factor')}")

                print("\nInitial Pose:")
                init_pose = agent_config.get('init_pose', {})
                print(f"- X: {init_pose.get('x')}")
                print(f"- Y: {init_pose.get('y')}")
                print(f"- Z: {init_pose.get('z')}")
                print(f"- H: {init_pose.get('h')}")

                print("\nGoals:")
                print(f"- Goal Radius: {agent_config.get('goal_radius')}")
                print(f"- Cyclic Goals: {agent_config.get('cyclic_goals')}")
                print(f"- Goals List: {agent_config.get('goals')}")

                # Print individual goal positions
                for goal_id in agent_config.get('goals', []):
                    goal_data = agent_config.get(goal_id, {})
                    print(f"\n  {goal_id}:")
                    print(f"  - X: {goal_data.get('x')}")
                    print(f"  - Y: {goal_data.get('y')}")
                    print(f"  - H: {goal_data.get('h')}")

            print("\n============================================")

            return params

    except Exception as e:
        print(f"Error loading config from {config_path}: {e}")
        raise


# load the hunav params
# DEFAULT_AGENT_CONFIG = load_config()

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
        inter_planner = str(
            obj.get("inter_planner", rosparam_get(str, "inter_planner", ""))
        )
        local_planner = str(
            obj.get("local_planner", rosparam_get(str, "local_planner", ""))
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
            model=model,
            agent=agent,
            record_data_dir=record_data,
            extra=obj,
        )


class DefaultParameter(typing.NamedTuple):
    value: typing.Any
