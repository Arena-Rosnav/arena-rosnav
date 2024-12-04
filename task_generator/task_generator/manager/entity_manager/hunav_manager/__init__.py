
import dataclasses
import enum
import os
import typing

import yaml
from task_generator.shared import DynamicObstacle, ModelWrapper, DynamicObstacleProps, PositionOrientation

from ament_index_python.packages import get_package_share_directory


@dataclasses.dataclass(frozen=True)
class HunavObstacleProps(DynamicObstacleProps):
    
    @dataclasses.dataclass(frozen=True)
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
    velocity: None
    desired_velocity: float
    radius: float
    linear_vel: float
    angular_vel: float
    behavior: Behavior
    goals: list
    cyclic_goals: bool
    goal_radius: float
    closest_obs: list

class HunavDynamicObstacle(HunavObstacleProps):

    class Behavior(HunavObstacleProps.Behavior):
        _default: typing.ClassVar["HunavObstacleProps.Behavior"] = HunavObstacleProps.Behavior(
            type=0,
            state=0,
            configuration=0,
            duration=0,
            once=False,
            vel=0.,
            dist=0.,
            social_force_factor=0.,
            goal_force_factor=0.,
            obstacle_force_factor=0.,
            other_force_factor=0.,
        )

        @classmethod
        def parse(cls, obj: dict) -> "HunavDynamicObstacle.Behavior":
            return cls(
                type=obj.get('type', cls._default.type),
                state = obj.get('state', cls._default.state),
                configuration=obj.get('configuration', cls._default.configuration),
                duration = obj.get('duration', cls._default.duration),
                once = obj.get('once', cls._default.once),
                vel = obj.get('vel', cls._default.vel),
                dist = obj.get('dist', cls._default.dist),
                social_force_factor = obj.get('social_force_factor', cls._default.social_force_factor),
                goal_force_factor = obj.get('goal_force_factor', cls._default.goal_force_factor),
                obstacle_force_factor = obj.get('obstacle_force_factor', cls._default.obstacle_force_factor),
                other_force_factor = obj.get('other_force_factor', cls._default.other_force_factor),
            )

    _default: typing.ClassVar["HunavObstacleProps"] = HunavObstacleProps(
        position=PositionOrientation(x=0,y=0,orientation=0),
        name='',
        model=ModelWrapper(''),
        extra={},
        waypoints=[],
        id=0,
        type=1,
        skin=0,
        group_id=0,
        yaw=0.,
        velocity=None,
        desired_velocity=0.,
        radius=0.,
        linear_vel=0.,
        angular_vel=0.,
        behavior=Behavior._default,
        goals=[],
        cyclic_goals=False,
        goal_radius=0.,
        closest_obs=[],
    )

    @classmethod
    def parse(cls, obj: dict, model: ModelWrapper) -> "HunavDynamicObstacle":

        base = DynamicObstacle.parse(obj, model)

        return cls(
            **dataclasses.asdict(base),
            id = obj.get("id", cls._default.id),
            behavior = cls.Behavior.parse(obj.get('behavior', {})),
            type = obj.get('type', cls._default.type),
            skin = obj.get('skin', cls._default.skin),
            group_id = obj.get('group_id', cls._default.group_id),
            yaw = obj.get('yaw', cls._default.yaw),
            velocity = None,
            desired_velocity = obj.get('max_vel', cls._default.desired_velocity),
            radius = obj.get('radius', cls._default.radius),
            linear_vel = cls._default.linear_vel,
            angular_vel = cls._default.angular_vel,
            goals = [],
            cyclic_goals = obj.get('cyclic_goals',cls._default.cyclic_goals),
            goal_radius = obj.get('goal_radius',cls._default.goal_radius),
            closest_obs = [],
        )


def _load_config(filename: str = "default.yaml") -> "HunavDynamicObstacle":
    """Load config from YAML file in arena_bringup configs."""

    # second priority: Install space
    config_path = os.path.join(
        get_package_share_directory("arena_bringup"),
        "configs",
        "hunav_agents",
        filename
    )

    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        agent_config = config['hunav_loader']['ros__parameters']['agent1']
        return HunavDynamicObstacle.parse(agent_config, ModelWrapper(''))
    
    except Exception as e:
        raise RuntimeError(f"Error loading config from {config_path}") from e

HunavDynamicObstacle._default = _load_config()
HunavDynamicObstacle.Behavior._default = HunavDynamicObstacle._default.behavior


# Animation configuration (from WorldGenerator)
SKIN_TYPES: dict[int, str] = {
    0: 'elegant_man.dae',
    1: 'casual_man.dae',
    2: 'elegant_woman.dae',
    3: 'regular_man.dae',
    4: 'worker_man.dae',
    5: 'walk.dae'
}

class ANIMATION_TYPES(str, enum.Enum):
    WALK = '07_01-walk.bvh',
    WALK_FORWARD = '69_02_walk_forward.bvh',
    NORMAL_WAIT = '137_28-normal_wait.bvh',
    WALK_CHILDISH = '142_01-walk_childist.bvh',
    SLOW_WALK = '07_04-slow_walk.bvh',
    WALK_SCARED = '142_17-walk_scared.bvh',
    WALK_ANGRY = '17_01-walk_with_anger.bvh'