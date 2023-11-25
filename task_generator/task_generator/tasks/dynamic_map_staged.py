import os
from typing import Optional

import rospy
from task_generator.constants import Constants
from task_generator.tasks.base_task import BaseTask
from task_generator.tasks.random import RandomTask
from task_generator.tasks.task_factory import TaskFactory
from task_generator.tasks.utils import (
    ITF_DynamicMap,
    RandomList,
    StageIndex,
    ITF_Staged,
)
from task_generator.utils import rosparam_get


@TaskFactory.register(Constants.TaskMode.DYNAMIC_MAP_STAGED)
class DynamicMapStagedTask(RandomTask):
    _static_obstacles: RandomList
    _interactive_obstacles: RandomList
    _dynamic_obstacles: RandomList

    itf_dynamicmap: ITF_DynamicMap
    itf_staged: ITF_Staged

    def __init__(
        self,
        curriculum_path: Optional[str] = None,
        config_path: Optional[str] = None,
        starting_stage: Optional[StageIndex] = None,
        debug_mode: Optional[bool] = None,
        **kwargs,
    ):
        if curriculum_path is None:
            curriculum_path = os.path.join(
                ITF_DynamicMap.CONFIG_PATH,
                rosparam_get(str, "configuration/task_mode/staged/curriculum"),
            )

        RandomTask.__init__(self, **kwargs)

        self.itf_staged = ITF_Staged(
            self,
            stages=ITF_Staged.read_file(curriculum_path),
            starting_index=starting_stage,
            training_config_path=config_path,
            debug_mode=debug_mode,
        )

        self.itf_staged.on_change_stage = lambda stage: self.on_change_stage(
            stage=stage
        )

        self.itf_dynamicmap = ITF_DynamicMap(
            self, configurations=ITF_DynamicMap.read_file(curriculum_path)
        )
        self.itf_dynamicmap.subscribe_reset(callback=self._cb_task_reset)

        self._eps_per_map = rosparam_get(float, "episode_per_map", 1.0)
        denominator: float = (
            rosparam_get(float, "num_envs", 1)
            if "eval_sim" not in self.robot_managers[0].namespace
            else 1.0
        )
        self._iterator = 1 / denominator

        self._episodes = 0

        self.itf_dynamicmap.update_config(arg=self.itf_staged.stage_index)

    def on_change_stage(self, stage: StageIndex):
        self.itf_dynamicmap.update_config(arg=stage)
        self.itf_dynamicmap.request_new_map()

        def callback():
            rospy.loginfo(
                f"({self.namespace}) Stage {self.itf_staged.stage_index}: Spawning {self.itf_staged.stage.static + self.itf_staged.stage.interactive} static and {self.itf_staged.stage.dynamic} dynamic obstacles!"
            )
            return False

        self.reset(callback=callback, stage=stage)

    @BaseTask.reset_helper(parent=RandomTask)
    def reset(
        self,
        reset_after_new_map: bool = False,
        first_map: bool = False,
        **kwargs,
    ):
        if first_map or self.itf_dynamicmap.episodes >= self._eps_per_map:
            self.itf_dynamicmap.request_new_map(first_map=first_map)
            return {}, None

        if not reset_after_new_map:
            # only update eps count when resetting the scene
            self.itf_dynamicmap.episodes += self._iterator

        return (
            dict(
                n_static_obstacles=self.itf_staged.stage.static,
                n_interactive_obstacles=self.itf_staged.stage.interactive,
                n_dynamic_obstacles=self.itf_staged.stage.dynamic,
            ),
            lambda: False,
        )

    def _cb_task_reset(self, *args, **kwargs):
        # task reset for all taskmanagers when one resets
        # update map manager
        self.itf_dynamicmap.update_map()
        self.reset(callback=lambda: None, reset_after_new_map=True)
