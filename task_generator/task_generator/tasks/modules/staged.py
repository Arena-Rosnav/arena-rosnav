import dataclasses
import os
from typing import Any, Callable, Dict, List, NamedTuple, Optional

import dynamic_reconfigure.client
import rospkg
import rospy
import std_msgs.msg as std_msgs
import yaml
from filelock import FileLock
from map_generator.constants import MAP_GENERATOR_NS
from task_generator.constants import Constants
from task_generator.shared import Namespace, rosparam_get
from task_generator.tasks.modules import TM_Module
from task_generator.tasks.task_factory import TaskFactory


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
    # map_properties: Dict[str, Any]

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

    PARAM_CONFIGURATION_NAME = lambda obs_type, param: f"RANDOM_{obs_type}_{param}"

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

            self.__config_lock = FileLock(f"{self.__training_config_path}.lock")

        self.__current_stage = -1

        self._dmre_client = dynamic_reconfigure.client.Client(
            name=self.NODE_CONFIGURATION, config_callback=self.reconfigure
        )

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
                    goal_radius = rosparam_get(float, self.PARAM_GOAL_RADIUS, 0.3)
                rospy.set_param(self.PARAM_GOAL_RADIUS, goal_radius)

                # set map generator params
                if self.stage.dynamic_map.algorithm is not None:
                    rospy.set_param(
                        MAP_GENERATOR_NS("algorithm"), self.stage.dynamic_map.algorithm
                    )
                if self.stage.dynamic_map.algorithm_config is not None:
                    rospy.set_param(
                        MAP_GENERATOR_NS("algorithm_config"),
                        self.stage.dynamic_map.algorithm_config,
                    )

                # set obstacle configuration
                obs_config = {}
                for obs_type in ["static", "dynamic", "interactive"]:
                    obs_config.update(
                        {
                            Mod_Staged.PARAM_CONFIGURATION_NAME(
                                obs_type, param
                            ): getattr(self.stage, obs_type)
                            for param in ["min", "max"]
                        }
                    )

                self._dmre_client.update_configuration(obs_config)

            # The current stage is stored inside the config file for when the training is stopped and later continued, the correct stage can be restored.
            if self.__training_config_path is not None:
                pass
                # self.__config_lock.acquire()

                # with open(self.__training_config_path, "r", encoding="utf-8") as target:
                #     config = yaml.load(target, Loader=yaml.FullLoader)
                #     config["callbacks"]["training_curriculum"][
                #         "curr_stage"
                #     ] = self.stage.serialize()

                # with open(self.__training_config_path, "w", encoding="utf-8") as target:
                #     yaml.dump(config, target, allow_unicode=True, indent=4)

                # self.__config_lock.release()

    def reconfigure(self, config):
        """
        Method called when the configuration is updated.

        This method updates the configuration based on the new values.

        Args:
            config: The new configuration values.
        """
        try:
            curriculum_file = str(self.CURRICULUM_PATH(config[self.PARAM_CURRICULUM]))
        except Exception as e:
            rospy.logwarn(e)
            curriculum_file = "default.yaml"

        assert os.path.isfile(curriculum_file), f"{curriculum_file} is not a file"

        with open(curriculum_file) as f:
            stages = {
                i: Stage(
                    static=stage.get("static", 0),
                    interactive=stage.get("interactive", 0),
                    dynamic=stage.get("dynamic", 0),
                    goal_radius=stage.get("goal_radius", None),
                    dynamic_map=DynamicMapStage(
                        algorithm=stage["map_generator"].get(
                            "algorithm"
                            # rosparam_get(str, MAP_GENERATOR_NS("algorithm")),
                        ),
                        algorithm_config=stage["map_generator"].get(
                            "algorithm_config"
                            # rosparam_get(dict, MAP_GENERATOR_NS("algorithm_config")),
                        ),
                        # map_properties=stage["map_generator"].get(
                        #     "map_properties",
                        #     rosparam_get(dict, MAP_GENERATOR_NS("map_properties")),
                        # ),
                    ),
                )
                for i, stage in enumerate(yaml.load(f, Loader=yaml.FullLoader))
            }

        try:
            starting_index = config[self.PARAM_INDEX]
        except Exception as e:
            rospy.logwarn(e)
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
        if (
            self.IS_EVAL_SIM and self.__current_stage != self.__target_stage
        ):  # TODO reconsider if this check is needed
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
