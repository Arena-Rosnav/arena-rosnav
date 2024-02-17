from task_generator.constants import Constants
from task_generator.tasks.modules import TM_Module
from task_generator.tasks.task_factory import TaskFactory

@TaskFactory.register_module(Constants.TaskMode.TM_Module.CLEAR_FORBIDDEN_ZONES)
class Mod_ClearForbiddenZones(TM_Module):
    def before_reset(self):
        self._TASK.world_manager.forbid_clear()