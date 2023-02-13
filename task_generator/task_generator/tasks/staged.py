import rospy
import os
from task_generator.constants import TaskMode
from task_generator.tasks.task_factory import TaskFactory
import yaml
import json
from std_msgs.msg import Bool
from filelock import FileLock

from task_generator.tasks.random import RandomTask


@TaskFactory.register(TaskMode.STAGED)
class StagedRandomTask(RandomTask):
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

    def __init__(
        self,
        obstacles_manager,
        robot_manager,
        map_manager,
        start_stage: int = 1,
        paths=None,
        namespace: str = "",
    ):
        super().__init__(obstacles_manager, robot_manager, map_manager)

        self.namespace = namespace
        self.namespace_prefix = f"/{namespace}/" if namespace else ""

        self._curr_stage = start_stage
        self._stages = {}
        self._stages = self._read_stages_from_file(paths.get("curriculum"))
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

    def _init_stage_and_update_config(self, stage):
        self._init_stage(stage)

        if self.namespace != "eval_sim":
            return

        rospy.set_param("/curr_stage", stage)
        rospy.set_param("/last_state_reached", stage == len(self._stages))

        self._update_stage_in_config(stage)

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
            yaml.dump(config, target, ensure_ascii=False, indent=4)

        self._config_lock.release()

    def reset(self):
        super().reset(
            static_obstacles=self._stages[self._curr_stage]["static"],
            dynamic_obstacles=self._stages[self._curr_stage]["dynamic"],
        )

    def _reset_robot_and_obstacles(
        self, static_obstacles=None, dynamic_obstacles=None, **kwargs
    ):
        super()._reset_robot_and_obstacles(
            static_obstacles=static_obstacles,
            dynamic_obstacles=dynamic_obstacles,
            **kwargs,
        )

    def _init_stage(self, stage):
        static_obstacles = self._stages[stage]["static"]
        dynamic_obstacles = self._stages[stage]["dynamic"]

        self._reset_robot_and_obstacles(
            static_obstacles=static_obstacles, dynamic_obstacles=dynamic_obstacles
        )

        rospy.loginfo(
            f"({self.namespace}) Stage {self._curr_stage}: Spawning {static_obstacles} static and {dynamic_obstacles} dynamic obstacles!"
        )

    def _check_start_stage(self, start_stage):
        assert isinstance(
            start_stage, int
        ), f"Given start stage {start_stage} is not an integer"

        assert start_stage >= 1 and start_stage <= len(self._stages), (
            "Start stage given for training curriculum out of bounds! Has to be between {1 to %d}!"
            % len(self._stages)
        )

    def _read_stages_from_file(self, path):
        assert os.path.isfile(path), f"{path} is not a file"

        with open(path, "r") as file:
            stages = yaml.load(file, Loader=yaml.FullLoader)

        assert isinstance(
            stages, dict
        ), f"{path} has the wrong format! Check the Docs to see the correct format."

        return stages
