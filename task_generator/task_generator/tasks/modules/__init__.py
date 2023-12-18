from task_generator.constants import Constants
from task_generator.tasks import Reconfigurable, Task

class TM_Module(Reconfigurable):

    _TASK: Task

    def __init__(self, task: Task, **kwargs):
        Reconfigurable.__init__(self)
        self._TASK = task

    def before_reset(self):
        ...

    def after_reset(self):
        ...