from typing import List, NamedTuple, Tuple
from task_generator.constants import Constants, TaskConfig
from task_generator.shared import DynamicObstacle, Obstacle
from task_generator.tasks import Props_, TaskMode

Obstacles = Tuple[List[Obstacle], List[DynamicObstacle]]

class TM_Obstacles(TaskMode):

    def __init__(self, **kwargs):
        TaskMode.__init__(self, **kwargs)

    def reset(self, **kwargs) -> Obstacles:
        return [], []