import dataclasses
import os
from typing import Any, Callable, Dict, List, NamedTuple, Optional

from filelock import FileLock
import rospkg
import yaml
import rospy
from task_generator.constants import Constants
from task_generator.shared import Namespace, rosparam_get
from task_generator.tasks.modules import TM_Module
from task_generator.tasks.task_factory import TaskFactory

import std_msgs.msg as std_msgs

import dynamic_reconfigure.client

class Stage(NamedTuple):
    static: int
    interactive: int
    dynamic: int
    goal_radius: Optional[float]

    def serialize(self) -> Dict:
        return self._asdict()

StageIndex = int
Stages = Dict[StageIndex, Stage]

@dataclasses.dataclass
class Config:
    stages: Stages
    starting_index: StageIndex

# StagedInterface
@TaskFactory.register_module(Constants.TaskMode.TM_Module.STAGED)
class Mod_Staged(TM_Module):

    __config: Config
    __target_stage: StageIndex
    __current_stage: StageIndex

    __training_config_path: Optional[Namespace]
    __debug_mode: bool
    __config_lock: FileLock

    PARAM_CURR_STAGE = "/curr_stage"
    PARAM_LAST_STAGE_REACHED = "/last_state_reached"
    PARAM_GOAL_RADIUS = "/goal_radius"
    PARAM_DEBUG_MODE = "debug_mode"

    PARAM_CURRICULUM = "STAGED_curriculum"
    PARAM_INDEX = "STAGED_index"

    TOPIC_PREVIOUS_STAGE = "previous_stage"
    TOPIC_NEXT_STAGE = "next_stage"

    CONFIG_PATH: Namespace
    CURRICULUM_PATH: Namespace

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.CONFIG_PATH = Namespace(
            os.path.join(
                rospkg.RosPack().get_path("arena_bringup"),
                "configs",
                "training"
            )
        )

        self.CURRICULUM_PATH = self.CONFIG_PATH("training_curriculums")

        self.__debug_mode = rosparam_get(bool, "debug_mode", False)

        if self.__debug_mode:
            self.__training_config_path = None

        def cb_next(*args, **kwargs):
            self.stage_index += 1

        rospy.Subscriber(
            os.path.join(
                Namespace(self._TASK.namespace).simulation_ns,
                self.TOPIC_NEXT_STAGE,
            ),
            std_msgs.Bool,
            cb_next,
        )

        def cb_previous(*args, **kwargs):
            self.stage_index -= 1

        rospy.Subscriber(
            os.path.join(
                Namespace(self._TASK.namespace).simulation_ns,
                self.TOPIC_PREVIOUS_STAGE,
            ),
            std_msgs.Bool,
            cb_previous,
        )

        if self.__training_config_path is not None:
            assert os.path.isfile(
                self.__training_config_path
            ), f"Found no 'training_config.yaml' at {self.__training_config_path}"

            self.__config_lock = FileLock(
                f"{self.__training_config_path}.lock")

        dynamic_reconfigure.client.Client(
            name=self.NODE_CONFIGURATION,
            config_callback=self.reconfigure
        )

        self.__current_stage = None

    def before_reset(self):

        if self.__current_stage != self.__target_stage:
            self.__current_stage = self.__target_stage
            rospy.loginfo(f"Loading stage {self.__current_stage}")

            # publish goal radius
            goal_radius = self.stage.goal_radius
            if goal_radius is None:
                goal_radius = rosparam_get(float, self.PARAM_GOAL_RADIUS, 0.3)
            rospy.set_param(self.PARAM_GOAL_RADIUS, goal_radius)

            # publish stage state
            if self.IS_EVAL_SIM:  # TODO reconsider if this check is needed
                rospy.set_param(self.PARAM_CURR_STAGE, self.__current_stage)
                rospy.set_param(self.PARAM_LAST_STAGE_REACHED, self.__current_stage == self.MAX_STAGE)

            # The current stage is stored inside the config file for when the training is stopped and later continued, the correct stage can be restored.
            if self.__training_config_path is not None:
                self.__config_lock.acquire()

                with open(self.__training_config_path, "r", encoding="utf-8") as target:
                    config = yaml.load(target, Loader=yaml.FullLoader)
                    config["callbacks"]["training_curriculum"]["curr_stage"] = self.stage.serialize()

                with open(self.__training_config_path, "w", encoding="utf-8") as target:
                    yaml.dump(config, target, allow_unicode=True, indent=4)

                self.__config_lock.release()


    def reconfigure(self, config):

        curriculum_file = str(self.CURRICULUM_PATH(rosparam_get(str, self.NODE_CONFIGURATION(self.PARAM_CURRICULUM))))
        assert os.path.isfile(curriculum_file), f"{curriculum_file} is not a file"

        with open(curriculum_file) as f:
            stages = {
            i: Stage(
                static=stage.get("static", 0),
                interactive=stage.get("interactive", 0),
                dynamic=stage.get("dynamic", 0),
                goal_radius=stage.get("goal_radius", None),
            )
            for i, stage in enumerate(yaml.load(f, Loader=yaml.FullLoader))
        }

        starting_index = rosparam_get(StageIndex, self.NODE_CONFIGURATION(self.PARAM_INDEX))

        self.__config = Config(
            stages = stages,
            starting_index = starting_index
        )

        self.stage_index = starting_index
        

    @property
    def IS_EVAL_SIM(self) -> bool:
        return self._TASK.namespace == "eval_sim"

    @property
    def MIN_STAGE(self) -> StageIndex:
        return 0

    @property
    def MAX_STAGE(self) -> StageIndex:
        return len(self.__config.stages) - 1

    @property
    def stage_index(self) -> StageIndex:
        """
        Current stage index.
        """
        return self.__current_stage

    @stage_index.setter
    def stage_index(self, val: StageIndex):

        val = val if val is not None else self.MIN_STAGE

        if val < self.MIN_STAGE or val > self.MAX_STAGE:
            rospy.loginfo(
                f"({self._TASK.namespace}) INFO: Tried to set stage {val} but was out of bounds [{self.MIN_STAGE}, {self.MAX_STAGE}]"
            )
            val = max(self.MIN_STAGE, min(self.MAX_STAGE, val))

        self.__target_stage = val

    @property
    def stage(self) -> Stage:
        """
        Current stage configuration.
        """
        return self.__config.stages[self.stage_index]