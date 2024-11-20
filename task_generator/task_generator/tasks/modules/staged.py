import dataclasses
import os
from typing import Any, Dict, List, NamedTuple, Optional

import rospkg
import std_msgs.msg as std_msgs
import yaml
from filelock import FileLock
from map_generator.constants import MAP_GENERATOR_NS
from task_generator.constants import Constants
from task_generator.shared import Namespace, rosparam_get
from task_generator.tasks.modules import TM_Module


class Stage(NamedTuple):
    static: int
    interactive: int
    dynamic: int
    goal_radius: Optional[float]
    dynamic_map: Optional["DynamicMapStage"]

    def serialize(self) -> Dict:
        return self._asdict()


class DynamicMapStage(NamedTuple):
    algorithm: str
    algorithm_config: Dict[str, Any]

    def serialize(self) -> Dict:
        return self._asdict()


StageIndex = int
Stages = Dict[StageIndex, Stage]


@dataclasses.dataclass
class Config:
    stages: Stages
    starting_index: StageIndex


class Mod_Staged(TM_Module):
    """
    A module for managing staged tasks in a task generator.
    Attributes:
        __config (Config): The configuration object for the staged tasks.
        __target_stage (StageIndex): The target stage index.
        __current_stage (StageIndex): The current stage index.
        __training_config_path (Optional[Namespace]): The path to the training configuration.
        __debug_mode (bool): Flag indicating whether debug mode is enabled.
        __config_lock (FileLock): The lock for the training configuration file.
        PARAM_CURR_STAGE (str): The parameter for the current stage index.
        PARAM_LAST_STAGE_REACHED (str): The parameter for the last stage reached flag.
        PARAM_GOAL_RADIUS (str): The parameter for the goal radius.
        PARAM_DEBUG_MODE (str): The parameter for the debug mode flag.
        PARAM_CURRICULUM (str): The parameter for the staged curriculum.
        PARAM_INDEX (str): The parameter for the staged index.
        TOPIC_PREVIOUS_STAGE (str): The topic for the previous stage.
        TOPIC_NEXT_STAGE (str): The topic for the next stage.
        CONFIG_PATH (Namespace): The path to the configuration files.
        CURRICULUM_PATH (Namespace): The path to the curriculum files.
    """
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

    def PARAM_CONFIGURATION_NAME(
        obs_type, param): return f"RANDOM_{obs_type}_{param}"

    TOPIC_PREVIOUS_STAGE = "previous_stage"
    TOPIC_NEXT_STAGE = "next_stage"

    CONFIG_PATH: Namespace
    CURRICULUM_PATH: Namespace

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.CONFIG_PATH = Namespace(
            os.path.join(
                rospkg.RosPack().get_path("arena_bringup"), "configs", "training"
            )
        )

        self.CURRICULUM_PATH = self.CONFIG_PATH("training_curriculums")

        self.__debug_mode = rosparam_get(bool, "debug_mode", False)

        self.__training_config_path = (
            rosparam_get(str, "training_config_path", None)
            if not self.__debug_mode
            else None
        )

        def cb_next(*args, **kwargs):
            self.stage_index += 1

        rospy.Subscriber(
            os.path.join(
                self._TASK.namespace,
                self.TOPIC_NEXT_STAGE,
            ),
            std_msgs.Bool,
            cb_next,
        )

        def cb_previous(*args, **kwargs):
            self.stage_index -= 1

        rospy.Subscriber(
            os.path.join(
                self._TASK.namespace,
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

        self.__current_stage = -1

    def before_reset(self):
        """
        Method called before resetting the module.
        This method updates the current stage and performs necessary actions before resetting the module.
        """
        if self.__current_stage != self.__target_stage:
            self.__current_stage = self.__target_stage
            rospy.loginfo(
                f"[{self._TASK.namespace}] Loading stage {self.__current_stage}"
            )
            # only update cpmfogiratopm with one task module instance
            if "sim_1" in rospy.get_name() or self.__debug_mode:
                # publish goal radius
                goal_radius = self.stage.goal_radius
                if goal_radius is None:
                    goal_radius = rosparam_get(
                        float, self.PARAM_GOAL_RADIUS, 0.3)
                rospy.set_param(self.PARAM_GOAL_RADIUS, goal_radius)
                # set map generator params
                if self.stage.dynamic_map.algorithm is not None:
                    rospy.set_param(
                        MAP_GENERATOR_NS(
                            "algorithm"), self.stage.dynamic_map.algorithm
                    )
                if self.stage.dynamic_map.algorithm_config is not None:
                    rospy.set_param(
                        MAP_GENERATOR_NS("algorithm_config"),
                        self.stage.dynamic_map.algorithm_config,
                    )
                # set obstacle configuration
                obs_config = {}
                for obs_type in ["static", "dynamic", "interactive"]:
                    for param in ["min", "max"]:
                        param_name = Mod_Staged.PARAM_CONFIGURATION_NAME(
                            obs_type, param)
                        param_value = getattr(self.stage, obs_type)
                        rospy.set_param(param_name, param_value)

            # The current stage is stored inside the config file for when the
            # training is stopped and later continued, the correct stage can be
            # restored.
            if self.__training_config_path is not None:
                pass

    def reconfigure(self, config):
        """
        Method called when the configuration is updated.
        This method updates the configuration based on the new values.
        Args:
            config: The new configuration values.
        """
        try:
            curriculum_file = str(self.CURRICULUM_PATH(
                config[self.PARAM_CURRICULUM]))
        except Exception as e:
            curriculum_file = "default.yaml"

        assert os.path.isfile(
            curriculum_file), f"{curriculum_file} is not a file"

        with open(curriculum_file) as f:
            stages = {
                i: Stage(
                    static=stage.get("static", 0),
                    interactive=stage.get("interactive", 0),
                    dynamic=stage.get("dynamic", 0),
                    goal_radius=stage.get("goal_radius", None),
                    dynamic_map=DynamicMapStage(
                        algorithm=stage["map_generator"].get("algorithm"),
                        algorithm_config=stage["map_generator"].get(
                            "algorithm_config"),
                    ),
                )
                for i, stage in enumerate(yaml.safe_load(f))
            }

        try:
            starting_index = config[self.PARAM_INDEX]
        except Exception as e:
            starting_index = 0

        self.__config = Config(stages=stages, starting_index=starting_index)

        self.stage_index = starting_index

    @property
    def IS_EVAL_SIM(self) -> bool:
        """
        Flag indicating whether the module is running in evaluation simulation mode.
        """
        return "eval_sim" in self._TASK.namespace

    @property
    def MIN_STAGE(self) -> StageIndex:
        """
        The minimum stage index.
        """
        return 0

    @property
    def MAX_STAGE(self) -> StageIndex:
        """
        The maximum stage index.
        """
        return len(self.__config.stages) - 1

    @property
    def stage_index(self) -> StageIndex:
        """
        Current stage index.
        """
        return self.__current_stage

    @stage_index.setter
    def stage_index(self, val: StageIndex):
        """
        Setter for the stage index.
        Args:
            val (StageIndex): The new stage index.
        """
        val = val if val is not None else self.MIN_STAGE

        if val < self.MIN_STAGE or val > self.MAX_STAGE:
            rospy.loginfo(
                f"({self._TASK.namespace}) INFO: Tried to set stage {val} but was out of bounds [{self.MIN_STAGE}, {self.MAX_STAGE}]"
            )
            val = max(self.MIN_STAGE, min(self.MAX_STAGE, val))

        self.__target_stage = val

        # publish stage state
        if self.IS_EVAL_SIM and self.__current_stage != self.__target_stage:
            rospy.set_param(self.PARAM_CURR_STAGE, self.__target_stage)
            rospy.set_param(
                self.PARAM_LAST_STAGE_REACHED,
                self.__target_stage == self.MAX_STAGE,
            )
            os.system(
                f"rosrun dynamic_reconfigure dynparam set /task_generator_server {self.PARAM_INDEX} {self.__target_stage}"
            )

    @property
    def stage(self) -> Stage:
        """
        Current stage configuration.
        """
        return self.__config.stages[self.stage_index]
