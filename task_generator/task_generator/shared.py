from __future__ import annotations
import collections
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
from pathlib import Path
import rospkg


import enum
import yaml
from rosros import rospify as rospy
from rosros.core import get_nodes
T = TypeVar("T")
U = TypeVar("U")
_unspecified = rospy.client._unspecified()
_UNSPECIFIED = rospy.client._unspecified
_notfound = object()


def rosparam_get(
    cast: Type[T], param_name: str, default: Union[U, _UNSPECIFIED] = _unspecified
) -> Union[T, U]:
    """
    Get typed ros parameter (strict)
    @cast: Return type of function
    @param_name: Name of parameter on parameter server
    @default: Default value. Raise ValueError is default is unset and parameter can't be found.
    """
    # if "task_generator_node" not in  get_nodes():
    return default
    val = rospy.get_param(param_name=param_name, default=_notfound)

    if val == _notfound:
        if isinstance(default, _UNSPECIFIED):
            raise ValueError(f"required parameter {param_name} is not set")
        return default

    try:
        return cast(val)
    except ValueError as e:
        raise ValueError(f"could not cast {val} to {cast} of param {param_name}") from e


class Namespace(str):
    def __call__(self, *args: str) -> Namespace:
        return Namespace(os.path.join(self, *args))

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
class ObstacleProps(EntityProps): ...


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
    # Directly use your absolute path
    config_path = f"/home/ahmo030/arena4_ws/src/arena/arena-rosnav/arena_bringup/configs/hunav_agents/{filename}" #fix (package path)
    
    print(f"Loading config from: {config_path}")
    
    # Open and load the file
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        params = config['hunav_loader']['ros__parameters']
        
        # Quick check of important values
        print(f"\nFound Agents: {params['agents']}")
        print(f"Agent1 type: {params['agent1']['behavior']['type']}")
        print(f"Agent1 radius: {params['agent1']['radius']}")
        
        return params



#load the hunav params     
DEFAULT_AGENT_CONFIG = load_config()

@dataclasses.dataclass(frozen=True)
class DynamicObstacle(DynamicObstacleProps):
    @dataclasses.dataclass
    class Behavior:
        type: int
        state: int
        configuration: int
        duration: float
        once: bool
        vel: float
        dist: float
        social_force_factor: float
        goal_force_factor: float
        obstacle_force_factor: float
        other_force_factor: float

    id: int
    type: int
    skin: int
    name: str
    group_id: int
    position: PositionOrientation
    yaw: float
    velocity: None  # oder entsprechender Typ
    desired_velocity: float
    radius: float
    linear_vel: float
    angular_vel: float
    behavior: Behavior
    goals: list
    cyclic_goals: bool
    goal_radius: float
    closest_obs: list
    model: ModelWrapper
    extra: dict

    @staticmethod
    def parse(obj: dict, model: ModelWrapper) -> "DynamicObstacle":
        # Parse behavior
        behavior_dict = obj.get('behavior', {})
        _type = behavior_dict.get('type', DEFAULT_AGENT_CONFIG['agent1']['behavior']['type'])
        _state = behavior_dict.get('state', DEFAULT_AGENT_CONFIG['agent1']['behavior']['state'])
        _conf = behavior_dict.get('configuration', DEFAULT_AGENT_CONFIG['agent1']['behavior']['configuration'])
        _duration = behavior_dict.get('duration', DEFAULT_AGENT_CONFIG['agent1']['behavior']['duration'])
        _once = behavior_dict.get('once', DEFAULT_AGENT_CONFIG['agent1']['behavior']['once'])
        _vel = behavior_dict.get('vel', DEFAULT_AGENT_CONFIG['agent1']['behavior']['vel'])
        _dist = behavior_dict.get('dist', DEFAULT_AGENT_CONFIG['agent1']['behavior']['dist'])
        _social_force = behavior_dict.get('social_force_factor', DEFAULT_AGENT_CONFIG['agent1']['behavior']['social_force_factor'])
        _goal_force = behavior_dict.get('goal_force_factor', DEFAULT_AGENT_CONFIG['agent1']['behavior']['goal_force_factor'])
        _obstacle_force = behavior_dict.get('obstacle_force_factor', DEFAULT_AGENT_CONFIG['agent1']['behavior']['obstacle_force_factor'])
        _other_force = behavior_dict.get('other_force_factor', DEFAULT_AGENT_CONFIG['agent1']['behavior']['other_force_factor'])

        # Parse basic properties
        name = str(obj.get("name", ""))
        position = PositionOrientation(*obj.get("init_pose", (0, 0, 0, 0)))
        _id = obj.get('id', DEFAULT_AGENT_CONFIG['agent1']['id'])
        _agent_type = obj.get('type', 1)  # Default to PERSON=1
        _skin = obj.get('skin', DEFAULT_AGENT_CONFIG['agent1']['skin'])
        _group_id = obj.get('group_id', DEFAULT_AGENT_CONFIG['agent1']['group_id'])
        _yaw = obj.get('yaw', 0.0)
        _velocity = None
        _desired_velocity = obj.get('max_vel', DEFAULT_AGENT_CONFIG['agent1']['max_vel'])
        _radius = obj.get('radius', DEFAULT_AGENT_CONFIG['agent1']['radius'])
        _linear_vel = 0.0
        _angular_vel = 0.0
        _goals = []
        _cyclic_goals = obj.get('cyclic_goals', DEFAULT_AGENT_CONFIG['agent1']['cyclic_goals'])
        _goal_radius = obj.get('goal_radius', DEFAULT_AGENT_CONFIG['agent1']['goal_radius'])
        _closest_obs = []

        return DynamicObstacle(
            id=_id,
            type=_agent_type,
            skin=_skin,
            name=name,
            group_id=_group_id,
            position=position,
            yaw=_yaw,
            velocity=_velocity,
            desired_velocity=_desired_velocity,
            radius=_radius,
            linear_vel=_linear_vel,
            angular_vel=_angular_vel,
            model=model,
            behavior=DynamicObstacle.Behavior(
                type=_type,
                state=_state,
                configuration=_conf,
                duration=_duration,
                once=_once,
                vel=_vel,
                dist=_dist,
                social_force_factor=_social_force,
                goal_force_factor=_goal_force,
                obstacle_force_factor=_obstacle_force,
                other_force_factor=_other_force
            ),
            goals=_goals,
            cyclic_goals=_cyclic_goals,
            goal_radius=_goal_radius,
            closest_obs=_closest_obs,
            extra=obj
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
            "record_data_dir", rospy.get_param("record_data_dir", None)
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
