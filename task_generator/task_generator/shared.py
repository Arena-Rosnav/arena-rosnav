from dataclasses import dataclass
from typing import Iterable, Optional, Tuple
from geometry_msgs.msg import Point, Pose
import enum

class ModelType(enum.Enum):
    UNKNOWN = ""
    URDF = "urdf"
    SDF = "sdf"
    YAML = "yaml"

@dataclass
class Model:
    type: ModelType
    description: str

@dataclass
class ObstacleDescription:
    name: str
    model: Model

@dataclass
class ObstacleDescriptionPose(ObstacleDescription):
    pose: Pose


ForbiddenZone = Tuple[float, float, float]

Waypoint = Tuple[float, float, float]

@dataclass
class CreatedObstacle:
    id: str
    position: Point

@dataclass
class CreatedStaticObstacle(CreatedObstacle):
    ...

@dataclass
class CreatedInteractiveObstacle(CreatedObstacle):
    ...

@dataclass
class CreatedDynamicObstacle(CreatedObstacle):
    waypoints: Iterable[Waypoint]