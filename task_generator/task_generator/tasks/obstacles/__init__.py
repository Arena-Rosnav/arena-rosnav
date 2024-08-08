from typing import List, Tuple
from task_generator.shared import DynamicObstacle, Obstacle
from task_generator.tasks import TaskMode

Obstacles = Tuple[List[Obstacle], List[DynamicObstacle]]

class TM_Obstacles(TaskMode):

    def __init__(self, **kwargs):
        TaskMode.__init__(self, **kwargs)

    def reset(self, **kwargs) -> Obstacles:
        return [], []