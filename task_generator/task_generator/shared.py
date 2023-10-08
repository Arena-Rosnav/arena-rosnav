from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Collection, Dict, Iterable, List, Optional, Tuple
from geometry_msgs.msg import Pose, Quaternion
import enum

EMPTY_LOADER = lambda *_, **__:Model(type=ModelType.UNKNOWN, name="", description="")

class ModelType(enum.Enum):
    UNKNOWN = ""
    URDF = "urdf"
    SDF = "sdf"
    YAML = "yaml"


@dataclass(frozen=True)
class Model:
    type: ModelType
    name: str
    description: str

BoundLoader = Callable[[Optional[Collection[ModelType]]], Model]

ForbiddenZone = Tuple[float, float, float]

PositionOrientation = Tuple[float, float, float]
Waypoint = Tuple[float, float, float]
Position = Tuple[float, float]

@dataclass
class _NoModel:
    model: BoundLoader

@dataclass
class _YesModel:
    model: Model


@dataclass
class ObstacleProps:
    position: PositionOrientation
    name: str
    extra: Dict

@dataclass
class DynamicObstacleProps(ObstacleProps):
    waypoints: List[Waypoint]

@dataclass
class RobotProps(ObstacleProps):
    planner: str
    namespace: str
    agent: str
    record_data: bool

@dataclass
class ObstacleSetup(ObstacleProps, _NoModel):
    @staticmethod
    def parse(obj: Dict, **kwargs) -> "ObstacleSetup":
        obstacle = ObstacleSetup(
            name=obj["name"],
            position=(float(obj["pos"][0]), float(obj["pos"][1]), 0),
            model=EMPTY_LOADER,
            **kwargs
        )
        obstacle.extra = obj
        return obstacle

@dataclass
class DynamicObstacleSetup(DynamicObstacleProps, _NoModel):
    @staticmethod
    def parse(obj: Dict, **kwargs) -> "DynamicObstacleSetup":
        obstacle = DynamicObstacleSetup(
            name=obj["name"],
            position=(float(obj["pos"][0]), float(obj["pos"][1]), 0),
            model=EMPTY_LOADER,
            waypoints=obj["waypoints"],
            **kwargs
        )
        obstacle.extra = obj
        return obstacle

@dataclass
class RobotSetup(RobotProps, _NoModel):
    @staticmethod
    def parse(obj: Dict, **kwargs) -> "RobotSetup":
        robot = RobotSetup(
            namespace=obj["namespace"],
            planner=obj["planner"],
            agent=obj["agent"],
            model=EMPTY_LOADER,
            record_data=False,
            **kwargs
        )
        return robot


@dataclass
class Obstacle(ObstacleProps, _YesModel):
    ...

@dataclass
class DynamicObstacle(DynamicObstacleProps, _YesModel):
    waypoints: Iterable[Waypoint]

@dataclass
class Robot(RobotProps, _YesModel):
    ...



@dataclass
class ScenarioObstacles:
    dynamic: List[DynamicObstacle]
    static: List[Obstacle]
    interactive: List[Obstacle]


ScenarioMap = str


@dataclass
class RobotGoal:
    start: PositionOrientation
    goal: PositionOrientation


@dataclass
class Scenario:
    obstacles: ScenarioObstacles
    map: ScenarioMap
    resets: int
    robots: List[RobotGoal]