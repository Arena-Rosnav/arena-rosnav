import os
from typing import Optional
from task_generator.constants import Constants


from task_generator.tasks.base_task import BaseTask
from task_generator.tasks.task_factory import TaskFactory
from task_generator.tasks.utils import Scenario, ITF_Scenario
from task_generator.utils import rosparam_get


@TaskFactory.register(Constants.TaskMode.SCENARIO)
class ScenarioTask(BaseTask):
    """
    Use Scenario
    """

    itf_scenario: ITF_Scenario

    def __init__(self, **kwargs):
        BaseTask.__init__(self, **kwargs)

        self.itf_scenario = ITF_Scenario(self)

        scenario_file_name: Optional[str] = rosparam_get(
            str, "~configuration/task_mode/scenario/scenario_file", "") or None

        if scenario_file_name is None:
            scenario = kwargs.get("scenario")
            assert isinstance(scenario, Scenario), "couldn't get scenario file"
            self._scenario = scenario
        else:
            with open(os.path.join(self.itf_scenario.CONFIG_PATH, scenario_file_name), "r") as file:
                self._scenario = self.itf_scenario.read_scenario_file(
                    file.read())

        self.itf_scenario.check_scenario(self._scenario)

    @BaseTask.reset_helper(parent=BaseTask)
    def reset(self, **kwargs):

        def callback():
            self.obstacle_manager.respawn(
                lambda: self.itf_scenario.setup_scenario(self._scenario))
            return False

        return {}, callback
