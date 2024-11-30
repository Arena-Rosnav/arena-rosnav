from task_generator.tasks import Task, TaskMode


class TM_Module(TaskMode):

    _TASK: Task

    def __init__(self, task: Task, **kwargs):
        TaskMode.__init__(self, task)
        self._TASK = task

    def before_reset(self):
        ...

    def after_reset(self):
        ...
