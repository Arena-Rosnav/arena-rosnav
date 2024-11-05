from dataclasses import dataclass
from .states import RobotState, TaskState

from .distributor import StateDistributor


@dataclass(frozen=False)
class SimulationStateContainer:
    robot: RobotState
    task: TaskState

    def distribute(self):
        StateDistributor(self).distribute()
