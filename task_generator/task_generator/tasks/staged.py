import functools
import time
from typing import Dict, Generator, List, Optional

import rospkg
import rospy
import os
from task_generator.constants import Constants
from task_generator.tasks.base_task import BaseTask
from task_generator.tasks.task_factory import TaskFactory
import yaml
from filelock import FileLock

from task_generator.tasks.random import RandomTask
from task_generator.constants import Constants
from task_generator.manager.map_manager import MapManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.tasks.utils import RandomInterface, RandomList, StageIndex, StagedInterface, Stages
from task_generator.utils import rosparam_get


@TaskFactory.register(Constants.TaskMode.STAGED)
class StagedRandomTask(BaseTask, RandomInterface, StagedInterface):
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

    _static_obstacles: RandomList
    _interactive_obstacles: RandomList
    _dynamic_obstacles: RandomList

    _namespace: str
    _stages: Stages
    _curr_stage: StageIndex

    def __init__(
        self,
        obstacle_manager: ObstacleManager,
        robot_managers: List[RobotManager],
        map_manager: MapManager,
        paths: Optional[Dict[str, str]] = None,
        start_stage: StageIndex = 1,
        namespace: str = "",
        **kwargs,
    ):
        BaseTask.__init__(
            self,
            obstacle_manager=obstacle_manager,
            robot_managers=robot_managers,
            map_manager=map_manager,
            **kwargs
        )

        allowed_obstacles = RandomInterface._load_obstacle_list(self)
        self._static_obstacles = allowed_obstacles.static
        self._interactive_obstacles = allowed_obstacles.interactive
        self._dynamic_obstacles = allowed_obstacles.dynamic

        self._namespace = namespace
        self.namespace_prefix = f"/{namespace}/" if namespace else ""

        # TODO rework this
        if paths is None:
            paths = dict(
                curriculum=os.path.join(rospkg.RosPack().get_path(
                    "training"), "configs", "training_curriculums", "default.yaml")
            )

        # TODO add mutexes for these against reset
        self._subscribe(
            self._namespace,
            cb_previous=functools.partial(self.previous_stage, self),
            cb_next=functools.partial(self.next_stage, self)
        )

        self._curr_stage = start_stage
        
        self._stages = self._read_stages_from_file(paths["curriculum"])
        self._debug_mode = rosparam_get(bool, "debug_mode", False)

        self._check_start_stage(self._stages, start_stage)
        self._publish_curr_stage(self._curr_stage)

        self._init_debug_mode(paths)
        self.reset(lambda:None)

    def next_stage(self, *args, **kwargs):
        if self._curr_stage >= len(self._stages):
            rospy.loginfo(
                f"({self._namespace}) INFO: Tried to trigger next stage but already reached last one"
            )
            return

        self._curr_stage = self._curr_stage + 1

        return self._init_stage_and_update_config(self._curr_stage)

    def previous_stage(self, *args, **kwargs):
        if self._curr_stage <= 1:
            rospy.loginfo(
                f"({self._namespace}) INFO: Tried to trigger previous stage but already reached first one"
            )
            return

        self._curr_stage = self._curr_stage - 1

        return self._init_stage_and_update_config(self._curr_stage)

    def _init_stage_and_update_config(self, stage: StageIndex):
        self.reset(lambda:None, stage=stage)

        if self._namespace != "eval_sim":
            return

        self._publish_curr_stage(stage=stage)
        self._publish_last_state_reached(stage == len(self._stages))

        # self._update_stage_in_config(stage)

        return stage

    def _init_debug_mode(self, paths):
        if self._debug_mode:
            return

        self._config_file_path = paths["config"]
        self._config_lock = FileLock(f"{self._config_file_path}.lock")

        assert os.path.isfile(
            self._config_file_path
        ), f"Found no 'training_config.yaml' at {self._config_file_path}"

    def _update_stage_in_config(self, stage):
        """
        The current stage is stored inside the config
        file for when the training is stopped and later
        continued, the correct stage can be restored.
        """
        if self._debug_mode:
            return

        self._config_lock.acquire()

        with open(self._config_file_path, "r", encoding="utf-8") as target:
            config = yaml.load(target, Loader=yaml.FullLoader)
            config["callbacks"]["training_curriculum"]["curr_stage"] = stage

        with open(self._config_file_path, "w", encoding="utf-8") as target:
            yaml.dump(config, target, allow_unicode=True, indent=4)

        self._config_lock.release()

    @BaseTask.reset_helper(parent=BaseTask)
    def reset(
        self,
        n_static_obstacles: Optional[int] = None,
        n_interactive_obstacles: Optional[int] = None, 
        n_dynamic_obstacles: Optional[int] = None,
        static_obstacles: Optional[RandomList] = None,
        interactive_obstacles: Optional[RandomList] = None,
        dynamic_obstacles: Optional[RandomList] = None,
        stage: Optional[StageIndex] = None,
        **kwargs):

        if stage is None:
            stage = self._curr_stage

        if n_static_obstacles is None:
            n_static_obstacles = self._stages[stage].static

        if n_interactive_obstacles is None:
            n_interactive_obstacles = self._stages[stage].interactive

        if n_dynamic_obstacles is None:
            n_dynamic_obstacles = self._stages[stage].dynamic

        if static_obstacles is None:
            static_obstacles = self._static_obstacles

        if interactive_obstacles is None:
            interactive_obstacles = self._interactive_obstacles

        if dynamic_obstacles is None:
            dynamic_obstacles = self._dynamic_obstacles

        if stage is None:
            stage = self._curr_stage

        goal_radius = self._stages[stage].goal_radius

        def callback():
            self._populate_goal_radius(goal_radius=goal_radius)

            rospy.loginfo(
                f"({self._namespace}) Stage {self._curr_stage}: Spawning {n_static_obstacles} static and {n_dynamic_obstacles} dynamic obstacles!"
            )

            self._obstacle_manager.respawn(callback=lambda:
            RandomInterface._setup_random(
                self,
                n_static_obstacles=n_static_obstacles,
                n_interactive_obstacles=n_interactive_obstacles,
                n_dynamic_obstacles=n_dynamic_obstacles,
                static_obstacles=static_obstacles,
                interactive_obstacles=interactive_obstacles,
                dynamic_obstacles=dynamic_obstacles
                )
            )
            time.sleep(1)

            return False

        return callback