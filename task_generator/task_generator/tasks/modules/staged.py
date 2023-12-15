
# StagedInterface


class Stage(NamedTuple):
    static: int
    interactive: int
    dynamic: int
    goal_radius: Optional[float]

    def serialize(self) -> Dict:
        return self._asdict()


StageIndex = int
Stages = Dict[StageIndex, Stage]


class ITF_Staged(ITF_Obstacle, ITF_Base):
    CONFIG_PATH = os.path.join(
        rospkg.RosPack().get_path("arena_bringup"),
        "configs",
        "training",
        "training_curriculums",
    )

    PARAM_CURR_STAGE = "/curr_stage"
    PARAM_LAST_STAGE_REACHED = "/last_state_reached"
    PARAM_GOAL_RADIUS = "/goal_radius"

    TOPIC_PREVIOUS_STAGE = "previous_stage"
    TOPIC_NEXT_STAGE = "next_stage"

    __stages: Stages
    __current_stage: StageIndex

    __training_config_path: Optional[str]
    __debug_mode: bool
    __config_lock: FileLock
    on_change_stage: Optional[Callable[[StageIndex], Any]]

    def __init__(
        self,
        TASK: Props_,
        stages: Stages,
        starting_index: Optional[StageIndex] = None,
        training_config_path: Optional[str] = None,
        debug_mode: Optional[bool] = None,
    ):
        ITF_Base.__init__(self, TASK=TASK)

        if starting_index is None:
            starting_index = rosparam_get(
                StageIndex, "~configuration/task_mode/staged/starting_index"
            )

        self.__stages = stages
        self.__current_stage = starting_index

        self.__training_config_path = training_config_path

        assert isinstance(
            self.stage_index, StageIndex
        ), f"Given start stage {starting_index} is invalid"

        if debug_mode is None:
            debug_mode = rosparam_get(bool, "debug_mode", False)
        self.__debug_mode = debug_mode

        self.__training_config_path = (
            None if self.__debug_mode else training_config_path
        )

        if self.__training_config_path is not None:
            assert os.path.isfile(
                self.__training_config_path
            ), f"Found no 'training_config.yaml' at {self.__training_config_path}"

            self.__config_lock = FileLock(
                f"{self.__training_config_path}.lock")

        self.on_change_stage = lambda stage: None

        def cb_next(*args, **kwargs):
            self.stage_index += 1

        rospy.Subscriber(
            os.path.join(
                Namespace(self.PROPS.namespace).simulation_ns,
                ITF_Staged.TOPIC_NEXT_STAGE,
            ),
            std_msgs.Bool,
            cb_next,
        )

        def cb_previous(*args, **kwargs):
            self.stage_index -= 1

        rospy.Subscriber(
            os.path.join(
                Namespace(self.PROPS.namespace).simulation_ns,
                ITF_Staged.TOPIC_PREVIOUS_STAGE,
            ),
            std_msgs.Bool,
            cb_previous,
        )

        self.stage_index = self.stage_index

    # TODO move to Stages
    @staticmethod
    def parse(config: List[Dict]) -> Stages:
        return {
            i: Stage(
                static=stage.get("static", 0),
                interactive=stage.get("interactive", 0),
                dynamic=stage.get("dynamic", 0),
                goal_radius=stage.get("goal_radius", None),
            )
            for i, stage in enumerate(config)
        }

    @staticmethod
    def read_file(path: str) -> Stages:
        assert os.path.isfile(path), f"{path} is not a file"

        with open(path, "r") as file:
            return ITF_Staged.parse(yaml.load(file, Loader=yaml.FullLoader))

    @property
    def IS_EVAL_SIM(self) -> bool:
        return self.PROPS.namespace == "eval_sim"

    @property
    def MIN_STAGE(self) -> StageIndex:
        return 0

    @property
    def MAX_STAGE(self) -> StageIndex:
        return len(self.__stages) - 1

    @property
    def stage_index(self) -> StageIndex:
        """
        Current stage index.
        """
        return self.__current_stage

    @stage_index.setter
    def stage_index(self, val: StageIndex):
        if val < self.MIN_STAGE or val > self.MAX_STAGE:
            rospy.loginfo(
                f"({self.PROPS.namespace}) INFO: Tried to set stage {val} but was out of bounds [{self.MIN_STAGE}, {self.MAX_STAGE}]"
            )
            return

        self.__current_stage = val

        # publish goal radius
        goal_radius = self.stage.goal_radius
        if goal_radius is None:
            goal_radius = rosparam_get(
                float, ITF_Staged.PARAM_GOAL_RADIUS, 0.3)
        rospy.set_param(ITF_Staged.PARAM_GOAL_RADIUS, goal_radius)

        # publish stage state
        if True or self.IS_EVAL_SIM:  # TODO reconsider if this check is needed
            rospy.set_param(ITF_Staged.PARAM_CURR_STAGE, val)
            rospy.set_param(ITF_Staged.PARAM_LAST_STAGE_REACHED,
                            val == self.MAX_STAGE)

        # The current stage is stored inside the config file for when the training is stopped and later continued, the correct stage can be restored.
        if self.__training_config_path is not None:
            self.__config_lock.acquire()

            with open(self.__training_config_path, "r", encoding="utf-8") as target:
                config = yaml.load(target, Loader=yaml.FullLoader)
                config["callbacks"]["training_curriculum"][
                    "curr_stage"
                ] = self.stage.serialize()

            with open(self.__training_config_path, "w", encoding="utf-8") as target:
                yaml.dump(config, target, allow_unicode=True, indent=4)

            self.__config_lock.release()

        # call callback
        if self.on_change_stage is not None:
            self.on_change_stage(self.stage_index)

    @property
    def stage(self) -> Stage:
        """
        Current stage configuration.
        """
        return self.__stages[self.stage_index]
