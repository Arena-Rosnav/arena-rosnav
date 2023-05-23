import os

import rospy
import yaml
from filelock import FileLock
from map_distance_server.srv import GetDistanceMap
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_msgs.msg import Bool, String
from task_generator.constants import Constants, TaskMode
from task_generator.tasks.task_factory import TaskFactory

from ..manager.map_manager import MapManager
from ..manager.obstacle_manager import ObstacleManager
from ..manager.robot_manager import RobotManager
from .dynamic_map_random import DynamicMapRandomTask
from .staged import StagedRandomTask


@TaskFactory.register(TaskMode.DYNAMIC_MAP_STAGED)
class DynamicMapStagedRandomTask(DynamicMapRandomTask):
    def __init__(
        self,
        obstacles_manager: ObstacleManager,
        robot_managers: RobotManager,
        map_manager: MapManager,
        start_stage: int = 1,
        paths=None,
        namespace: str = "",
        *args,
        **kwargs,
    ):
        super().__init__(
            obstacles_manager, robot_managers, map_manager, *args, **kwargs
        )

        self.namespace = namespace
        self.namespace_prefix = f"/{namespace}/" if namespace else ""

        self._curr_stage = start_stage
        self._stages = self._read_stages_from_file(paths["curriculum"])
        self._debug_mode = rospy.get_param("debug_mode", False)

        self._check_start_stage(start_stage)
        rospy.set_param("/curr_stage", self._curr_stage)
        self._init_debug_mode(paths)

        self._sub_next = rospy.Subscriber(
            f"{self.namespace_prefix}next_stage", Bool, self.next_stage
        )
        self._sub_previous = rospy.Subscriber(
            f"{self.namespace_prefix}previous_stage", Bool, self.previous_stage
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
            return

        rospy.set_param("/curr_stage", stage)
        rospy.set_param("/last_state_reached", stage == len(self._stages))

        # self._update_stage_in_config(stage)

        self.reset(first_map=True)

        return stage

    def _init_stage(self, stage: int) -> int:
        self.populate_map_info(stage)
        self._populate_goal_radius(stage)
        static_obstacles = self._stages[stage]["static"]
        dynamic_obstacles = self._stages[stage]["dynamic"]

        self._reset_robot_and_obstacles(
            static_obstacles=static_obstacles, dynamic_obstacles=dynamic_obstacles
        )

        rospy.loginfo(
            f"({self.namespace}) Stage {self._curr_stage}: Spawning {static_obstacles} static and {dynamic_obstacles} dynamic obstacles!"
        )

    def reset(
        self,
        start=None,
        goal=None,
        static_obstacles=None,
        dynamic_obstacles=None,
        reset_after_new_map=False,
        first_map=False,
    ):
        # set obstacle num according to current stage
        super().reset(
            start=start,
            goal=goal,
            static_obstacles=static_obstacles,
            dynamic_obstacles=dynamic_obstacles,
            reset_after_new_map=reset_after_new_map,
            first_map=first_map,
        )

    def next_stage(self, _):
        if self._curr_stage >= len(self._stages):
            rospy.loginfo(
                f"({self.namespace}) INFO: Tried to trigger next stage but already reached last one"
            )
            return

        self._curr_stage = self._curr_stage + 1

        return self._init_stage_and_update_config(self._curr_stage)

    def previous_stage(self, _):
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
                yaml.dump(config, target, ensure_ascii=False, indent=4)
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

    def populate_map_info(self, stage: int):
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
