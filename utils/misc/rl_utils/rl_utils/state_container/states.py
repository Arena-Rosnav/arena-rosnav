from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class LaserState:
    attach_full_range_laser: bool
    laser_num_beams: int
    laser_max_range: float


@dataclass(frozen=True)
class VelocityState:
    min_linear_vel: float
    max_linear_vel: float
    min_angular_vel: float
    max_angular_vel: float
    min_translational_vel: Optional[float] = None
    max_translational_vel: Optional[float] = None


@dataclass(frozen=True)
class ActionState:
    is_discrete: bool
    actions: list
    velocity_state: VelocityState
    is_holonomic: Optional[bool] = False


@dataclass(frozen=True)
class RobotState:
    radius: float
    safety_distance: float
    action_state: ActionState
    laser_state: LaserState


@dataclass(frozen=True)
class SemanticState:
    num_ped_types: int
    ped_min_speed_x: float
    ped_max_speed_x: float
    ped_min_speed_y: float
    ped_max_speed_y: float
    social_state_num: int


@dataclass(frozen=True)
class TaskModuleState:
    tm_robots: str
    tm_obstacles: str
    tm_modules: str


@dataclass(frozen=False)
class TaskState:
    goal_radius: float
    max_steps: int
    semantic_state: SemanticState
    task_modules: TaskModuleState
