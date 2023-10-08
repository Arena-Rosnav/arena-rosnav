from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Tuple
from geometry_msgs.msg import Point, Pose, Quaternion
import enum

class ModelType(enum.Enum):
    UNKNOWN = ""
    URDF = "urdf"
    SDF = "sdf"
    YAML = "yaml"

@dataclass
class Model:
    type: ModelType
    name: str
    description: str

ForbiddenZone = Tuple[float, float, float]

Position = Tuple[float, float, float]
Waypoint = Tuple[float, float, float]

@dataclass
class ObstacleConfig:
    model: Model
    position: Optional[Position] = None
    
@dataclass
class DynamicObstacleConfig(ObstacleConfig):
    waypoints: Optional[List[Waypoint]] = None



@dataclass
class Obstacle:
    name: str
    pose: Pose
    model: Model
    extra: Dict = field(default_factory=lambda:dict(), init=False)

    @staticmethod
    def parse(obj: Any):
        obstacle = Obstacle(
            name=obj["name"],
            pose=Pose(
                position=obj["pos"],
                orientation=Quaternion(0,0,0,1)
            ),
            model=Model(type=ModelType.UNKNOWN, name="", description=""),
        )
        obstacle.extra = obj
        return obstacle

@dataclass
class DynamicObstacle(Obstacle):
    waypoints: Iterable[Waypoint]

    @staticmethod
    def parse(obj: Any):
        obstacle = DynamicObstacle(
            name=obj["name"],
            pose=Pose(
                position=obj["pos"],
                orientation=Quaternion(0,0,0,1)
            ),
            model=Model(type=ModelType.UNKNOWN, name="", description=""),
            waypoints=obj["waypoints"]
        )
        obstacle.extra = obj
        return obstacle


@dataclass
class ScenarioObstacles:
    dynamic: List[DynamicObstacle]
    static: List[Obstacle]
    interactive: List[Obstacle]

ScenarioMap = str

@dataclass
class RobotGoal:
    start: Position
    goal: Position

@dataclass
class Scenario:
    obstacles: ScenarioObstacles
    map: ScenarioMap
    resets: int
    robots: List[RobotGoal]