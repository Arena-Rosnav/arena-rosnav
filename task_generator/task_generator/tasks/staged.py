from typing import Optional

import rospy
import os
from task_generator.constants import Constants
from task_generator.tasks.base_task import BaseTask
from task_generator.tasks.random import RandomTask
from task_generator.tasks.task_factory import TaskFactory

from task_generator.constants import Constants
from task_generator.tasks.utils import StageIndex, ITF_Staged
from task_generator.utils import rosparam_get


@TaskFactory.register(Constants.TaskMode.STAGED)
class StagedTask(RandomTask):
    """
    The staged task mode is designed for the trainings
    process of arena-rosnav. In general, it behaves
    like the random task mode but there are multiple
    stages between one can switch. Between the stages,
    the amount of static and dynamic obstacles changes.
    The amount of obstacles is defined in a curriculum
    file, the path to said file is a key in the `paths`
    parameter.
    """

    itf_staged: ITF_Staged

    def __init__(
        self,
        curriculum_path: Optional[str] = None,
        config_path: Optional[str] = None,
        starting_stage: Optional[StageIndex] = None,
        debug_mode: Optional[bool] = None,
        **kwargs,
    ):
        RandomTask.__init__(self, **kwargs)

        if curriculum_path is None:
            curriculum_path = os.path.join(
                ITF_Staged.CONFIG_PATH,
                rosparam_get(str, "configuration/task_mode/staged/curriculum"),
            )

        self.itf_staged = ITF_Staged(
            self,
            stages=ITF_Staged.read_file(curriculum_path),
            starting_index=starting_stage,
            training_config_path=config_path,
            debug_mode=debug_mode,
        )

        self.itf_staged.on_change_stage = lambda stage: self.reset(
            callback=lambda: None, stage=stage, **kwargs)

    @BaseTask.reset_helper(parent=RandomTask)
    def reset(self, stage: Optional[StageIndex] = None, **kwargs):
        if stage is None:
            stage = self.itf_staged.stage_index

        return (
            dict(
                n_static_obstacles=self.itf_staged.stage.static,
                n_interactive_obstacles=self.itf_staged.stage.interactive,
                n_dynamic_obstacles=self.itf_staged.stage.dynamic,
            ),
            lambda: False,
        )

    def on_change_stage(self, *args, **kwargs):
        def callback():
            rospy.loginfo(
                f"({self.namespace}) Stage {self.itf_staged.stage_index}: Spawning {self.itf_staged.stage.static + self.itf_staged.stage.interactive} static and {self.itf_staged.stage.dynamic} dynamic obstacles!"
            )
            return False

        callback()
