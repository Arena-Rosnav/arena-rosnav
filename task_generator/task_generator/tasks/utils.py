from dataclasses import asdict, dataclass
import json
import os
import random
import sys
from typing import Any, Callable, Dict, Generator, List, Optional, Set, Tuple

import numpy as np
import yaml

import rospy
import rospkg
import rospy
from task_generator.constants import Constants
from task_generator.shared import DynamicObstacle, ModelWrapper, Obstacle, PositionOrientation, Waypoint

from task_generator.manager.map_manager import MapManager
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.utils import ModelLoader, rosparam_get

from std_msgs.msg import Bool

import xml.etree.ElementTree as ET

from nav_msgs.msg import OccupancyGrid
from map_distance_server.srv import GetDistanceMapResponse
from std_msgs.msg import String

class ManagerProps:
    _obstacle_manager: ObstacleManager
    _robot_managers: List[RobotManager]
    _map_manager: MapManager

class ModelloaderProps:
    _model_loader: ModelLoader
    _dynamic_model_loader: ModelLoader

# ObstacleInterface

class ObstacleInterface(ManagerProps):
    """
    Helper methods to fill partially initialized obstacles
    """

    def _create_dynamic_obstacle(self, waypoints: Optional[List[Waypoint]] = None, **kwargs) -> DynamicObstacle:
        """
            Create dynamic obstacle from partial params.
            @name: Name of the obstacle
            @model: ModelWrapper of models
            @position: (optional) Starting position
            @waypoints: (optional) List of waypoints
            @extra: (optional) Extra properties to store
        """

        setup = self._create_obstacle(**kwargs)

        if waypoints is None:

            safe_distance = 0.5
            # the first waypoint
            waypoints = [(*setup.position[:2], safe_distance)]
            safe_distance = 0.1  # the other waypoints don't need to avoid robot
            for j in range(1):
                dist = 0
                while dist < 8:
                    [x2, y2, *
                        _] = self._map_manager.get_random_pos_on_map(safe_distance)
                    dist = np.linalg.norm(
                        [waypoints[-1][0] - x2, waypoints[-1][1] - y2])
                    waypoints.append((x2, y2, 1))

        return DynamicObstacle(**{
            **asdict(setup),
            **dict(waypoints=waypoints)
        })

    def _create_obstacle(self, name: str, model: ModelWrapper, position: Optional[PositionOrientation] = None, extra: Optional[Dict] = None, **kwargs) -> Obstacle:
        """ 
        Create non-dynamic obstacle from partial params.
        @name: Name of the obstacle
        @model: ModelWrapper of models
        @position: (optional) Starting position
        @extra: (optional) Extra properties to store
        """

        safe_distance = 0.5

        if position is None:
            point: Waypoint = self._map_manager.get_random_pos_on_map(
                safe_distance)
            position = (point[0], point[1], np.pi * np.random.random())

        if extra is None:
            extra = dict()

        return Obstacle(
            position=position,
            name=name,
            model=model,
            extra=extra,
            **kwargs
        )


# ScenarioInterface

@dataclass
class ScenarioObstacles:
    dynamic: List[DynamicObstacle]
    static: List[Obstacle]
    interactive: List[Obstacle]

@dataclass
class ScenarioMap:
    yaml: object
    xml: ET.ElementTree
    path: str

@dataclass
class RobotGoal:
    start: PositionOrientation
    goal: PositionOrientation

@dataclass
class Scenario:
    obstacles: ScenarioObstacles
    map: ScenarioMap
    robots: List[RobotGoal]

class ScenarioInterface(ManagerProps, ModelloaderProps):
    """
    Helper methods to handle scenarios
    """

    SCENARIO_BASE_PATH = os.path.join(rospkg.RosPack().get_path("arena_bringup"), "configs", "scenarios")

    def _read_scenario_file(self, scenario_file_content: str) -> Scenario:
        """
        Parse scenario file content as Scenario object.
        @scenario_file_content: string content of scenario file
        """

        scenario_file = json.loads(scenario_file_content)

        static_obstacles = [Obstacle.parse(obs, model=self._model_loader.bind(
            obs["model"])) for obs in scenario_file["obstacles"]["static"]]
        interactive_obstacles = []
        dynamic_obstacles = [DynamicObstacle.parse(obs, model=self._dynamic_model_loader.bind(
            obs["model"])) for obs in scenario_file["obstacles"]["dynamic"]]


        base_path = rospkg.RosPack().get_path("arena-simulation-setup")

        map_name = scenario_file["map"]

        map_path = os.path.join(
            base_path,
            "maps",
            map_name,
            "map.yaml"
        )

        with open(map_path) as f:
            map_yaml = yaml.load(f, Loader=yaml.FullLoader)

        with open(os.path.join(base_path, "worlds", map_name, "ped_scenarios", f"{map_name}.xml")) as f:
            map_xml = ET.parse(f)

        scenario = Scenario(
            obstacles=ScenarioObstacles(
                static=static_obstacles,
                interactive=interactive_obstacles,
                dynamic=dynamic_obstacles
            ),
            map=ScenarioMap(yaml=map_yaml, path=map_path, xml=map_xml),
            robots=[RobotGoal(start=robot["start"], goal=robot["goal"])
                    for robot in scenario_file["robots"]]
        )

        self._check_scenario(scenario=scenario)

        return scenario
    
    def _check_scenario(self, scenario: Scenario) -> bool:

        #check map path
        static_map: str = str(rospy.get_param("map_path", ""))
        scenario_map_path = scenario.map.path

        if static_map != scenario_map_path:
            rospy.logerr(
                "Map path of scenario and static map are not the same. Shutting down.")
            rospy.logerr(f"Scenario Map Path {scenario_map_path}")
            rospy.logerr(f"Static Map Path {static_map}")

            rospy.signal_shutdown(
                "Map path of scenario and static map are not the same.")
            sys.exit()
            return False


        #check robot manager length
        scenario_robots_length = len(scenario.robots)
        setup_robot_length = len(self._robot_managers)

        if setup_robot_length > scenario_robots_length:
            self._robot_managers = self._robot_managers[:scenario_robots_length]
            rospy.logwarn(
                "Roboto setup contains more robots than the scenario file.")

        if scenario_robots_length > setup_robot_length:
            scenario.robots = scenario.robots[:setup_robot_length]
            rospy.logwarn("Scenario file contains more robots than setup.")

        return True

    def _setup_scenario(self, scenario: Scenario):

        self._obstacle_manager.spawn_map_obstacles(scenario.map.xml)
        self._obstacle_manager.spawn_obstacles(scenario.obstacles.static)
        self._obstacle_manager.spawn_obstacles(
            scenario.obstacles.interactive)
        self._obstacle_manager.spawn_dynamic_obstacles(
            scenario.obstacles.dynamic)
        
        for index, robot in enumerate(scenario.robots):
            if index >= len(self._robot_managers):
                break

            manager = self._robot_managers[index]

            manager.reset(start_pos=robot.start[:2], goal_pos=robot.goal[:2])
            manager.move_robot_to_pos(pos=robot.start[:2])

    

# RandomInterface

RandomRange = Tuple[int, int]

@dataclass
class RandomObstacleRanges:
    static: RandomRange
    interactive: RandomRange
    dynamic: RandomRange

RandomList = Dict[str, float]

@dataclass
class RandomObstacleList:
    static: RandomList
    interactive: RandomList
    dynamic: RandomList

class RandomInterface(ObstacleInterface, ManagerProps, ModelloaderProps):

    def _load_obstacle_list(self) -> RandomObstacleList:
        def str_to_RandomList(value: list) -> RandomList:
            #TODO optional probability weighting of models in config
            return dict.fromkeys(value, 1)


        return RandomObstacleList(
            static=str_to_RandomList(value=rosparam_get(list, "~configuration/task_mode/random/static/models", self._model_loader.models)),
            interactive=str_to_RandomList(value=rosparam_get(list, "~configuration/task_mode/random/interactive/models", self._model_loader.models)),
            dynamic=str_to_RandomList(value=rosparam_get(list, "~configuration/task_mode/random/dynamic/models", self._dynamic_model_loader.models))
        )

    def _load_obstacle_ranges(self) -> RandomObstacleRanges:
        return RandomObstacleRanges(
            static=(
                int(str(rosparam_get(int, "~configuration/task_mode/random/static/min", Constants.Random.MIN_STATIC_OBS))),
                int(str(rosparam_get(int, "~configuration/task_mode/random/static/max", Constants.Random.MAX_STATIC_OBS)))
            ),
            interactive=(
                int(str(rosparam_get(int, "~configuration/task_mode/random/interactive/min", Constants.Random.MIN_INTERACTIVE_OBS))),
                int(str(rosparam_get(int, "~configuration/task_mode/random/interactive/max", Constants.Random.MAX_INTERACTIVE_OBS)))
            ),
            dynamic=(
                int(str(rosparam_get(int, "~configuration/task_mode/random/dynamic/min", Constants.Random.MIN_DYNAMIC_OBS))),
                int(str(rosparam_get(int, "~configuration/task_mode/random/dynamic/max", Constants.Random.MAX_DYNAMIC_OBS)))
            )
        )

    @staticmethod
    def _randrange_generator(r_range: RandomRange) -> Generator[int, None, None]:
        range_min, range_max = r_range
        range_max = max(range_min, range_max)
        while True:
            yield random.randint(range_min, range_max)

    def _setup_random(
        self,
        n_static_obstacles: int,
        n_interactive_obstacles: int,
        n_dynamic_obstacles: int,
        static_obstacles: RandomList,
        interactive_obstacles: RandomList,
        dynamic_obstacles: RandomList
        ):

        robot_positions: List[Waypoint] = []  # may be needed in the future idk

        for manager in self._robot_managers:

            start_pos = self._map_manager.get_random_pos_on_map(
                manager.safe_distance)
            goal_pos = self._map_manager.get_random_pos_on_map(
                manager.safe_distance, forbidden_zones=[start_pos])

            manager.reset(start_pos=start_pos[:2], goal_pos=goal_pos[:2])

            robot_positions.append(start_pos)
            robot_positions.append(goal_pos)

        self._obstacle_manager.reset()
        self._map_manager.init_forbidden_zones()

        # Create static obstacles
        if n_static_obstacles:
            self._obstacle_manager.spawn_obstacles([
                self._create_obstacle(name=model, model=self._model_loader.bind(model))
                for model in random.choices(population=list(static_obstacles.keys()), weights=list(static_obstacles.values()), k=n_static_obstacles)
            ])

        # Create interactive obstacles
        if n_interactive_obstacles:
            self._obstacle_manager.spawn_obstacles([
                self._create_obstacle(name=model, model=self._model_loader.bind(model))
                for model in random.choices(population=list(interactive_obstacles.keys()), weights=list(interactive_obstacles.values()), k=n_interactive_obstacles)
            ])

        # Create dynamic obstacles
        if n_dynamic_obstacles:
            self._obstacle_manager.spawn_dynamic_obstacles([
                self._create_dynamic_obstacle(name=model, model=self._dynamic_model_loader.bind(model))
                for model in random.choices(population=list(dynamic_obstacles.keys()), weights=list(dynamic_obstacles.values()), k=n_dynamic_obstacles)
            ])

        return False, (0, 0, 0)


# StagedInterface

@dataclass
class Stage:
    static: int
    interactive: int
    dynamic: int
    goal_radius: Optional[float]

StageIndex = int
Stages = Dict[StageIndex, Stage]


class StagedInterface(ObstacleInterface, ManagerProps, ModelloaderProps):

    # TODO move to Stages
    @staticmethod
    def parse(config: Dict[int, Dict]) -> Stages:
        return {
            i:Stage(
                static=stage.get("static", 0),
                interactive=stage.get("interactive", 0),
                dynamic=stage.get("dynamic", 0),
                goal_radius=stage.get("goal_radius")
            )
            for i, stage in config.items()
        }

    def _subscribe(self, namespace:str, cb_previous:Callable, cb_next:Callable):
        sub_next_stage = rospy.Subscriber(
            f"{namespace}next_stage", Bool, cb_next
        )
        sub_prev_stage = rospy.Subscriber(
            f"{namespace}previous_stage", Bool, cb_previous
        )
        return sub_next_stage, sub_prev_stage

    def _publish_curr_stage(self, stage: StageIndex):
        rospy.set_param("/curr_stage", stage)

    def _publish_last_state_reached(self, reached: bool):
        rospy.set_param("/last_state_reached", reached)

    def _check_start_stage(self, stages: Stages, start_stage: StageIndex):
        assert isinstance(
            start_stage, int
        ), f"Given start stage {start_stage} is not an integer"

        assert start_stage >= 1 and start_stage <= len(stages), (
            "Start stage given for training curriculum out of bounds! Has to be between {1 to %d}!"
            % len(stages)
        )

    def _read_stages_from_file(self, path: str) -> Stages:
        assert os.path.isfile(path), f"{path} is not a file"

        with open(path, "r") as file:
            return StagedInterface.parse(yaml.load(file, Loader=yaml.FullLoader))

    def _populate_goal_radius(self, goal_radius: Optional[float]):
        if goal_radius is None:
            goal_radius = rosparam_get(float, "/goal_radius", 0.3)

        rospy.set_param("/goal_radius", goal_radius)

class DynamicMapInterface(ManagerProps):
    def update_map(self, dist_map: GetDistanceMapResponse):
        self._map_manager.update_map(dist_map)

    def get_service(self):
        # requests distance map from distance map server
        get_dist_map_service = rospy.ServiceProxy("/distance_map", GetDistanceMapResponse)

        return get_dist_map_service

    def create_publishers(self):
        # requests new map from map generator
        map_request_pub = rospy.Publisher(
            "/request_new_map", String, queue_size=1)
        # task reset for all taskmanagers when one resets
        task_reset_pub = rospy.Publisher(
            "/dynamic_map/task_reset", String, queue_size=1
        )

        return map_request_pub, task_reset_pub
        
    def subscribe(self, callback: Callable):
        self.task_reset_sub = rospy.Subscriber(
            "/dynamic_map/task_reset", String, callback
        )

    def request_new_map(self, first_map: bool, request_pub: rospy.Publisher, reset_pub: rospy.Publisher):
        # set current eps immediately to 0 so that only one task
        # requests a new map
        if not first_map:
            rospy.set_param("/dynamic_map/curr_eps", 0)

        request_pub.publish("")

        rospy.wait_for_message("/map", OccupancyGrid)
        rospy.wait_for_message("/signal_new_distance_map", String)

        reset_pub.publish("")

        rospy.loginfo("===================")
        rospy.loginfo("+++ Got new map +++")
        rospy.loginfo("===================")

    def increment_episodes(self, step: float):
        new_count = rosparam_get(int, "/dynamic_map/curr_eps", 0)
        new_count += step
        rospy.set_param("/dynamic_map/curr_eps", new_count)