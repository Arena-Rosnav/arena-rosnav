from dataclasses import dataclass
from .states import RobotState, TaskState


@dataclass(frozen=False)
class SimulationStateContainer:
    robot: RobotState
    task: TaskState
