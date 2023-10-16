import os
from typing import Dict, List, Optional

import rospkg

import rospy
import yaml
from filelock import FileLock
from std_msgs.msg import Bool
from task_generator.constants import Constants
from task_generator.manager.map_manager import MapManager
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.tasks.dynamic_map_random import DynamicMapRandomTask
from task_generator.tasks.task_factory import TaskFactory


@TaskFactory.register(Constants.TaskMode.DYNAMIC_MAP_STAGED)
class DynamicMapStagedRandomTask(DynamicMapRandomTask):
    def __init__(
        self,
        obstacle_manager: ObstacleManager,
        robot_managers: List[RobotManager],
        map_manager: MapManager,
        paths: Optional[Dict[str, str]] = None,
        start_stage: int = 1,
        namespace: str = "",
        **kwargs,
    ):
        super().__init__(obstacle_manager=obstacle_manager,
                         robot_managers=robot_managers, map_manager=map_manager, **kwargs)

        #TODO refactor this into multiple inheriatance together with staged.py, random.py, dynamic_map_random.py
        self.namespace = namespace
        self.namespace_prefix = f"/{namespace}/" if namespace else ""

        #TODO rework this
        if paths is None:
            paths = dict(
                curriculum=os.path.join(rospkg.RosPack().get_path(
                    "training"), "configs", "training_curriculums", "default.yaml")
            )

        self._curr_stage = start_stage
        self._stages = self._read_stages_from_file(paths["curriculum"])
        self._debug_mode = rospy.get_param("debug_mode", False)

        self._check_start_stage(start_stage)
        rospy.set_param("/curr_stage", self._curr_stage)
        self._init_debug_mode(paths)

        self._sub_next = rospy.Subscriber(
            f"{self.namespace_prefix}next_stage", Bool, self._next_stage
        )
        self._sub_previous = rospy.Subscriber(
            f"{self.namespace_prefix}previous_stage", Bool, self._previous_stage
        )

        self._init_stage(self._curr_stage)

    def _read_stages_from_file(self, path: str) -> dict:
        assert os.path.isfile(path), f"{path} is not a file"

        with open(path, "r") as file:
            stages = yaml.load(file, Loader=yaml.FullLoader)

        assert isinstance(
            stages, dict
        ), f"{path} has the wrong format! Check the documentation to see the correct format."

        return stages

    def _init_debug_mode(self, paths):
        if self._debug_mode:
            return

        self._config_file_path = paths["config"]
        self._config_lock = FileLock(f"{self._config_file_path}.lock")

        assert os.path.isfile(
            self._config_file_path
        ), f"Found no 'training_config.yaml' at {self._config_file_path}"

    def _init_stage_and_update_config(self, stage: int) -> int:
        self._init_stage(stage)

        if self.namespace != "eval_sim":
            return -1

        rospy.set_param("/curr_stage", stage)
        rospy.set_param("/last_state_reached", stage == len(self._stages))

        # self._update_stage_in_config(stage)

        self.reset(first_map=True)

        return stage

    def _init_stage(self, stage: int, **kwargs):
        self._populate_map_info(stage)
        self._populate_goal_radius(stage)
        static_obstacles = self._stages[stage]["static"]
        dynamic_obstacles = self._stages[stage]["dynamic"]

        super().reset(
            static_obstacles=static_obstacles,
            dynamic_obstacles=dynamic_obstacles,
            **kwargs
        )

        rospy.loginfo(
            f"({self.namespace}) Stage {self._curr_stage}: Spawning {static_obstacles} static and {dynamic_obstacles} dynamic obstacles!"
        )

    def reset(
        self,
        **kwargs
    ):
        # set obstacle num according to current stage
        self._init_stage(self._curr_stage, **kwargs)

    def _next_stage(self, _):
        if self._curr_stage >= len(self._stages):
            rospy.loginfo(
                f"({self.namespace}) INFO: Tried to trigger next stage but already reached last one"
            )
            return

        self._curr_stage = self._curr_stage + 1

        return self._init_stage_and_update_config(self._curr_stage)

    def _previous_stage(self, _):
        if self._curr_stage <= 1:
            rospy.loginfo(
                f"({self.namespace}) INFO: Tried to trigger previous stage but already reached first one"
            )
            return

        self._curr_stage = self._curr_stage - 1

        return self._init_stage_and_update_config(self._curr_stage)

    def _update_stage_in_config(self, stage):
        """
        The current stage is stored inside the config
        file for when the training is stopped and later
        continued, the correct stage can be restored.
        """
        if self._debug_mode:
            return

        self._config_lock.acquire()
        try:
            with open(self._config_file_path, "r", encoding="utf-8") as target:
                config = yaml.load(target, Loader=yaml.FullLoader)
                config["callbacks"]["training_curriculum"]["curr_stage"] = stage

            with open(self._config_file_path, "w", encoding="utf-8") as target:
                yaml.dump(config, target, allow_unicode=True, indent=4)
        except Exception as e:
            pass
        self._config_lock.release()

    def _check_start_stage(self, start_stage):
        assert isinstance(
            start_stage, int
        ), f"Given start stage {start_stage} is not an integer"

        assert start_stage >= 1 and start_stage <= len(self._stages), (
            "Start stage given for training curriculum out of bounds! Has to be between {1 to %d}!"
            % len(self._stages)
        )

    def _populate_map_info(self, stage: int):
        stage_cfg = self._stages[stage]
        generator = rospy.get_param("generator")

        log = f"({self.namespace}) Stage {self._curr_stage}: Setting [Map Generator: {generator}] parameters"
        for key, value in stage_cfg["map_generator"][generator].items():
            log += f"\t{key}={value}"
            rospy.set_param(f"/generator_configs/{generator}/{key}", value)

        rospy.loginfo(log)

    def _populate_goal_radius(self, stage: int):
        try:
            goal_radius = self._stages[stage]["goal_radius"]
        except KeyError:
            goal_radius = rospy.get_param("/goal_radius", 0.3)

        rospy.set_param("/goal_radius", goal_radius)
