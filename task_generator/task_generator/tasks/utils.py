import dataclasses
import itertools
import json
import math
import os
import random
import sys
from typing import (
    Any,
    Callable,
    Dict,
    Generator,
    Iterator,
    List,
    NamedTuple,
    Optional,
    Tuple,
    Union,
    overload,
)
import cv2
from filelock import FileLock

import numpy as np
import yaml

import rospy
import rospkg
import rospy
from task_generator.constants import Constants
from task_generator.manager.utils import WorldMap
from task_generator.shared import (
    DynamicObstacle,
    ModelWrapper,
    Namespace,
    Obstacle,
    PositionOrientation,
    PositionRadius,
)

from task_generator.tasks.base_task import Props_
from task_generator.utils import rosparam_get

import std_msgs.msg as std_msgs
import nav_msgs.msg as nav_msgs
import map_distance_server.srv as map_distance_server_srvs


class ITF_Base:
    PROPS: Props_

    def __init__(self, TASK: Props_, **kwargs):
        self.PROPS = TASK


# ObstacleInterface


class ITF_Obstacle(ITF_Base):
    """
    Helper methods to fill partially initialized obstacles
    """

    def __init__(self, TASK: Props_):
        ITF_Base.__init__(self, TASK=TASK)

    def create_dynamic_obstacle(
        self,
        waypoints: Optional[List[PositionRadius]] = None,
        n_waypoints: int = 2,
        **kwargs
    ) -> DynamicObstacle:
        """
        Create dynamic obstacle from partial params.
        @name: Name of the obstacle
        @model: ModelWrapper of models
        @position: (optional) Starting position
        @waypoints: (optional) List of waypoints
        @extra: (optional) Extra properties to store
        """

        setup = self.create_obstacle(**kwargs)

        if waypoints is None:

            waypoints = [PositionRadius(setup.position.x, setup.position.y, 1)]
            safe_distance = 0.1  # the other waypoints don't need to avoid robot

            waypoints += [PositionRadius(*pos, 1) for pos in self.PROPS.world_manager.get_positions_on_map(n=n_waypoints, safe_dist=safe_distance)]

        return DynamicObstacle(**{
            **dataclasses.asdict(setup),
            **dict(waypoints=waypoints)
        })


    def create_obstacle(
        self,
        name: str,
        model: ModelWrapper,
        position: Optional[PositionOrientation] = None,
        extra: Optional[Dict] = None,
        **kwargs,
    ) -> Obstacle:
        """
        Create non-dynamic obstacle from partial params.
        @name: Name of the obstacle
        @model: ModelWrapper of models
        @position: (optional) Starting position
        @extra: (optional) Extra properties to store
        """

        safe_distance = 1

        if position is None:
            point = self.PROPS.world_manager.get_position_on_map(safe_distance)
            position = PositionOrientation(point.x, point.y, random.random() * 2*np.pi)

        if extra is None:
            extra = dict()

        return Obstacle(
            position=position, name=name, model=model, extra=extra, **kwargs
        )


# ScenarioInterface


@dataclasses.dataclass
class ScenarioObstacles:
    dynamic: List[DynamicObstacle]
    static: List[Obstacle]
    interactive: List[Obstacle]


@dataclasses.dataclass
class ScenarioMap:
    yaml: Dict  # TODO type this with a schema
    occupancy: np.ndarray
    path: str


class RobotGoal(NamedTuple):
    start: PositionOrientation
    goal: PositionOrientation


@dataclasses.dataclass
class Scenario:
    obstacles: ScenarioObstacles
    map: ScenarioMap
    robots: List[RobotGoal]


class ITF_Scenario(ITF_Base):
    """
    Helper methods to handle scenarios
    """

    def __init__(self, TASK: Props_):
        ITF_Base.__init__(self, TASK=TASK)

    CONFIG_PATH = os.path.join(
        rospkg.RosPack().get_path("arena_bringup"), "configs", "scenarios"
    )

    def read_scenario_file(self, scenario_file_content: str) -> Scenario:
        """
        Parse scenario file content as Scenario object.
        @scenario_file_content: string content of scenario file
        """

        scenario_file = json.loads(scenario_file_content)

        static_obstacles = [
            Obstacle.parse(
                obs,
                model=self.PROPS.model_loader.bind(obs["model"])
            )
            for obs in scenario_file["obstacles"]["static"]
        ]
        interactive_obstacles = [
            Obstacle.parse(
                obs,
                model=self.PROPS.model_loader.bind(obs["model"])
            )
            for obs in scenario_file["obstacles"]["interactive"]
        ]
        dynamic_obstacles = [
            DynamicObstacle.parse(
                obs,
                model=self.PROPS.dynamic_model_loader.bind(obs["model"])
            )
            for obs in scenario_file["obstacles"]["dynamic"]
        ]

        base_path = rospkg.RosPack().get_path("arena-simulation-setup")
        map_name = scenario_file["map"]
        map_path = os.path.join(
            base_path,
            "maps",
            map_name
        )
        map_file = os.path.join(map_path, "map.yaml")

        with open(map_file) as f:
            map_yaml = yaml.load(f, Loader=yaml.FullLoader)

        map_img = cv2.imread(os.path.join(
            map_path, map_yaml.get("image")), cv2.IMREAD_GRAYSCALE)

        scenario = Scenario(
            obstacles=ScenarioObstacles(
                static=static_obstacles,
                interactive=interactive_obstacles,
                dynamic=dynamic_obstacles,
            ),
            map=ScenarioMap(yaml=map_yaml, path=map_file, occupancy=map_img),
            robots=[RobotGoal(start=PositionOrientation(*robot["start"]), goal=PositionOrientation(*robot["goal"]))
                    for robot in scenario_file["robots"]]
        )

        self.check_scenario(scenario=scenario)

        return scenario

    def check_scenario(self, scenario: Scenario) -> bool:
        # check map path
        static_map: str = rosparam_get(str, "map_path", "")
        scenario_map_path = scenario.map.path

        if static_map != scenario_map_path:
            rospy.logerr(
                "Map path of scenario and static map are not the same. Shutting down."
            )
            rospy.logerr(f"Scenario Map Path {scenario_map_path}")
            rospy.logerr(f"Static Map Path {static_map}")

            rospy.signal_shutdown(
                "Map path of scenario and static map are not the same."
            )
            sys.exit()
            return False

        # check robot manager length
        scenario_robots_length = len(scenario.robots)
        setup_robot_length = len(self.PROPS.robot_managers)

        if setup_robot_length > scenario_robots_length:
            self.PROPS.robot_managers = self.PROPS.robot_managers[
                :scenario_robots_length
            ]
            rospy.logwarn(
                "Roboto setup contains more robots than the scenario file.")

        if scenario_robots_length > setup_robot_length:
            scenario.robots = scenario.robots[:setup_robot_length]
            rospy.logwarn("Scenario file contains more robots than setup.")

        return True

    def setup_scenario(self, scenario: Scenario):

        # self.PROPS.world_manager.update_world(
        #     world_map=WorldMap(
        #         occupancy=WorldLayers(
        #             walls=WorldOccupancy.from_map(scenario.map.occupancy)),
        #         origin=Position(*scenario.map.yaml.get("origin", (0, 0, 0))[:2]),
        #         resolution=float(scenario.map.yaml.get("resolution", 1.0)),
        #         time=genpy.Time(0)
        #     )
        # )

        self.PROPS.obstacle_manager.spawn_world_obstacles(
            self.PROPS.world_manager.world)
        self.PROPS.obstacle_manager.spawn_obstacles(
            scenario.obstacles.static + scenario.obstacles.interactive)
        self.PROPS.obstacle_manager.spawn_dynamic_obstacles(
            scenario.obstacles.dynamic)

        for index, scenario_robot in enumerate(scenario.robots):
            if index >= len(self.PROPS.robot_managers):
                break

            robot = self.PROPS.robot_managers[index]

            robot.reset(start_pos=scenario_robot.start,
                        goal_pos=scenario_robot.goal)


# RandomInterface
RandomRange = Tuple[int, int]


class RandomObstacleRanges(NamedTuple):
    static: RandomRange
    interactive: RandomRange
    dynamic: RandomRange


RandomList = Dict[str, float]


class RandomObstacleList(NamedTuple):
    static: RandomList
    interactive: RandomList
    dynamic: RandomList


class ITF_Random(ITF_Obstacle, ITF_Base):
    def __init__(self, TASK: Props_):
        ITF_Base.__init__(self, TASK=TASK)

    def load_obstacle_list(self) -> RandomObstacleList:
        def str_to_RandomList(value: list) -> RandomList:
            # TODO optional probability weighting of models in config
            return dict.fromkeys(value, 1)

        return RandomObstacleList(
            static=str_to_RandomList(
                value=rosparam_get(
                    list,
                    "~configuration/task_mode/random/static/models",
                    self.PROPS.model_loader.models,
                )
            ),
            interactive=str_to_RandomList(
                value=rosparam_get(
                    list,
                    "~configuration/task_mode/random/interactive/models",
                    self.PROPS.model_loader.models,
                )
            ),
            dynamic=str_to_RandomList(
                value=rosparam_get(
                    list,
                    "~configuration/task_mode/random/dynamic/models",
                    self.PROPS.dynamic_model_loader.models,
                )
            ),
        )

    def load_obstacle_ranges(self) -> RandomObstacleRanges:
        return RandomObstacleRanges(
            static=(
                int(
                    str(
                        rosparam_get(
                            int,
                            "~configuration/task_mode/random/static/min",
                            Constants.Random.MIN_STATIC_OBS,
                        )
                    )
                ),
                int(
                    str(
                        rosparam_get(
                            int,
                            "~configuration/task_mode/random/static/max",
                            Constants.Random.MAX_STATIC_OBS,
                        )
                    )
                ),
            ),
            interactive=(
                int(
                    str(
                        rosparam_get(
                            int,
                            "~configuration/task_mode/random/interactive/min",
                            Constants.Random.MIN_INTERACTIVE_OBS,
                        )
                    )
                ),
                int(
                    str(
                        rosparam_get(
                            int,
                            "~configuration/task_mode/random/interactive/max",
                            Constants.Random.MAX_INTERACTIVE_OBS,
                        )
                    )
                ),
            ),
            dynamic=(
                int(
                    str(
                        rosparam_get(
                            int,
                            "~configuration/task_mode/random/dynamic/min",
                            Constants.Random.MIN_DYNAMIC_OBS,
                        )
                    )
                ),
                int(
                    str(
                        rosparam_get(
                            int,
                            "~configuration/task_mode/random/dynamic/max",
                            Constants.Random.MAX_DYNAMIC_OBS,
                        )
                    )
                ),
            ),
        )

    @staticmethod
    def randrange_generator(r_range: RandomRange) -> Generator[int, None, None]:
        range_min, range_max = r_range
        range_max = max(range_min, range_max)
        while True:
            yield random.randint(range_min, range_max)

    def setup_random(
        self,
        n_static_obstacles: int,
        n_interactive_obstacles: int,
        n_dynamic_obstacles: int,
        static_obstacles: RandomList,
        interactive_obstacles: RandomList,
        dynamic_obstacles: RandomList,
        robot_positions: Optional[List[Tuple[PositionOrientation, PositionOrientation]]] = None,
    ):
        if robot_positions is None:
            robot_positions = []

        self.PROPS.world_manager.forbid_clear()

        if len(robot_positions) < len(self.PROPS.robot_managers):
            generated_positions = [
                PositionOrientation(position.x, position.y, random.random() * 2*np.pi)
                for position in (
                    self.PROPS.world_manager.get_positions_on_map(
                        n=2*(len(self.PROPS.robot_managers) - len(robot_positions)),
                        safe_dist=max(robot.safe_distance for robot in self.PROPS.robot_managers)
                    )
                )
            ]

            robot_positions += list(zip(generated_positions[::2], generated_positions[1::2]))

        for robot, pos in zip(self.PROPS.robot_managers, robot_positions):
            robot.reset(start_pos=pos[0], goal_pos=pos[1])

        # self.PROPS.obstacle_manager.reset()
        self.PROPS.obstacle_manager.spawn_world_obstacles(self.PROPS.world_manager.world)

        def indexer() -> Callable[..., int]:
            indices: Dict[str, Iterator[int]] = dict()

            def index(model: str):
                if model not in indices:
                    indices[model] = itertools.count(1)
                return next(indices[model])
            return index

        waypoints_per_ped = 2
        points = self.PROPS.world_manager.get_positions_on_map(n=n_static_obstacles + n_dynamic_obstacles + n_dynamic_obstacles*(1+waypoints_per_ped), safe_dist=0.1)

        positions = itertools.cycle(PositionOrientation(*pos, 2*np.pi * random.random()) for pos in points[:(n_static_obstacles + n_dynamic_obstacles + n_dynamic_obstacles)])
        waypoints = itertools.cycle(PositionRadius(*pos, 1) for pos in points[(n_static_obstacles + n_dynamic_obstacles + n_dynamic_obstacles):])

        obstacles = []

        # Create static obstacles
        if n_static_obstacles:
            index = indexer()

            obstacles += [
                ITF_Obstacle.create_obstacle(
                    self,
                    name=f"S_{model}_{index(model)}",
                    model=self.PROPS.model_loader.bind(model),
                    position=next(positions)
                )
                for model in random.choices(
                    population=list(static_obstacles.keys()),
                    weights=list(static_obstacles.values()),
                    k=n_static_obstacles,
                )
            ]

        # Create interactive obstacles
        if n_interactive_obstacles:
            index = indexer()

            obstacles += [
                ITF_Obstacle.create_obstacle(
                    self,
                    name=f"I_{model}_{index(model)}",
                    model=self.PROPS.model_loader.bind(model),
                    position=next(positions)
                )
                for model in random.choices(
                    population=list(interactive_obstacles.keys()),
                    weights=list(interactive_obstacles.values()),
                    k=n_interactive_obstacles,
                )
            ]

        self.PROPS.obstacle_manager.spawn_obstacles(obstacles)

        # Create dynamic obstacles
        if n_dynamic_obstacles:
            index = indexer()

            self.PROPS.obstacle_manager.spawn_dynamic_obstacles(
                [
                    ITF_Obstacle.create_dynamic_obstacle(
                        self,
                        name=f"D_{model}_{index(model)}",
                        model=self.PROPS.dynamic_model_loader.bind(model),
                        waypoints=list(itertools.islice(waypoints, waypoints_per_ped)),
                        position=next(positions),
                    )
                    for model in random.choices(
                        population=list(dynamic_obstacles.keys()),
                        weights=list(dynamic_obstacles.values()),
                        k=n_dynamic_obstacles,
                    )
                ]
            )

        return False, (0, 0, 0)


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


# DYNAMIC MAP INTERFACE
DynamicMapIndex = StageIndex

DynamicMapConfiguration = Dict[str, Dict]
DynamicMapConfigurations = Dict[DynamicMapIndex, DynamicMapConfiguration]


class DynamicMapPublishers(NamedTuple):
    map_request_pub: rospy.Publisher
    task_reset_pub: rospy.Publisher


class ITF_DynamicMap(ITF_Base):
    CONFIG_PATH = ITF_Staged.CONFIG_PATH

    PARAM_MAP_FILE = "map_file"
    PARAM_EPISODES = "/dynamic_map/curr_eps"
    PARAM_GENERATOR = "generator"
    PARAM_GENERATOR_CONFIGS = "/generator_configs"

    TOPIC_REQUEST_MAP = "/request_new_map"
    TOPIC_RESET = "/dynamic_map/task_reset"
    TOPIC_MAP = "/map"
    TOPIC_SIGNAL_MAP = "/signal_new_distance_map"

    SERVICE_DISTANCE_MAP = "/distance_map"

    __map_request_pub: rospy.Publisher
    __task_reset_pub: rospy.Publisher
    __get_dist_map_service: rospy.ServiceProxy

    __configurations: DynamicMapConfigurations

    def __init__(
        self,
        TASK: Props_,
        configurations: DynamicMapConfigurations
    ):

        ITF_Base.__init__(self, TASK=TASK)

        self.__check_dynamic_map()

        self.__configurations = configurations

        # requests new map from map generator
        self.__map_request_pub = rospy.Publisher(
            ITF_DynamicMap.TOPIC_REQUEST_MAP, std_msgs.String, queue_size=1
        )
        # task reset for all taskmanagers when one resets
        self.__task_reset_pub = rospy.Publisher(
            ITF_DynamicMap.TOPIC_RESET, std_msgs.String, queue_size=1
        )

        self.__get_dist_map_service = rospy.ServiceProxy(
            ITF_DynamicMap.SERVICE_DISTANCE_MAP, map_distance_server_srvs.GetDistanceMap
        )

    @staticmethod
    def parse(config: List[Dict]) -> DynamicMapConfigurations:
        return {i: stage.get("map_generator", dict()) for i, stage in enumerate(config)}

    @staticmethod
    def read_file(path: str) -> DynamicMapConfigurations:
        assert os.path.isfile(path), f"{path} is not a file"

        with open(path, "r") as file:
            return ITF_DynamicMap.parse(yaml.load(file, Loader=yaml.FullLoader))

    @staticmethod
    def const_config(config: DynamicMapConfiguration) -> DynamicMapConfigurations:
        class ConstConfig(dict):
            def __getitem__(self, __key: Any) -> DynamicMapConfiguration:
                return config

        return ConstConfig()

    def __check_dynamic_map(self):
        map_name = rosparam_get(str, ITF_DynamicMap.PARAM_MAP_FILE, "")
        if map_name != Constants.MapGenerator.MAP_FOLDER_NAME:
            raise ValueError(
                f"'DYNAMIC_MAP_RANDOM' task can only be used with dynamic map, otherwise the MapGenerator isn't used. (expected: {Constants.MapGenerator.MAP_FOLDER_NAME}, got: {map_name})"
            )

    def update_map(self, dist_map: Optional[map_distance_server_srvs.GetDistanceMapResponse] = None):
        if dist_map is None:
            dist_map = self.__get_dist_map_service()

        if isinstance(dist_map, map_distance_server_srvs.GetDistanceMapResponse):
            self.PROPS.world_manager.update_world(
                world_map=WorldMap.from_distmap(distmap=dist_map))

    def subscribe_reset(self, callback: Callable) -> rospy.Subscriber:
        return rospy.Subscriber(ITF_DynamicMap.TOPIC_RESET, std_msgs.String, callback)

    def request_new_map(self, first_map: bool = False):
        # set current eps immediately to 0 so that only one task
        # requests a new map
        if not first_map:
            self.episodes = 0

        self.__map_request_pub.publish("")

        rospy.wait_for_message(ITF_DynamicMap.TOPIC_MAP, nav_msgs.OccupancyGrid)
        rospy.wait_for_message(ITF_DynamicMap.TOPIC_SIGNAL_MAP, std_msgs.String)

        self.__task_reset_pub.publish("")

        rospy.loginfo("===================")
        rospy.loginfo("+++ Got new map +++")
        rospy.loginfo("===================")

    @property
    def episodes(self) -> float:
        return rosparam_get(float, ITF_DynamicMap.PARAM_EPISODES, 0)

    @episodes.setter
    def episodes(self, value: float):
        rospy.set_param(ITF_DynamicMap.PARAM_EPISODES, value)

    def get_config(self, index: DynamicMapIndex) -> DynamicMapConfiguration:
        return self.__configurations.get(index, dict())

    @overload
    def update_config(self, arg: DynamicMapIndex):
        ...

    @overload
    def update_config(self, arg: DynamicMapConfiguration):
        ...

    def update_config(
        self,
        arg: Union[DynamicMapIndex, DynamicMapConfiguration],
        generator: Optional[str] = None,
    ):
        config: DynamicMapConfiguration
        index: Any

        if isinstance(arg, DynamicMapIndex):
            config = self.get_config(index=arg)
            index = arg
        # TODO switch this to DynamicMapConfiguation when it gets class def
        elif isinstance(arg, dict):
            config = arg
            index = "inline"
        else:
            raise ValueError()

        if generator is None:
            generator = rosparam_get(str, ITF_DynamicMap.PARAM_GENERATOR)

        log = f"({self.PROPS.namespace}) Stage {index}: Setting [Map Generator: {generator}] parameters"

        config_generator = config.get(generator, dict())

        for key, value in config_generator.items():
            log += f"\t{key}={value}"
            rospy.set_param(
                f"{ITF_DynamicMap.PARAM_GENERATOR_CONFIGS}/{generator}/{key}", value
            )

        rospy.loginfo(log)
