from typing import (
    TYPE_CHECKING,
    Any,
    Callable,
    Dict,
    List,
    Literal,
    NewType,
    Optional,
    Tuple,
    Type,
    TypeVar,
    Union,
)
import gymnasium
from rl_utils.utils.paths import Path, PathComponent

# Gym Env
EnvironmentType = TypeVar(
    "EnvironmentType", bound=Union[gymnasium.Env, gymnasium.Wrapper]
)
InformationDict = Dict[str, Any]

PathsDict = Dict[Type[PathComponent], PathComponent]

CustomDiscreteAction = Dict[str, Union[str, float]]
CustomDiscreteActionList = List[CustomDiscreteAction]

ObservationCollectorDataClass = TypeVar("ObservationCollectorDataClass")

T = TypeVar("T")
ProcessingFunction = Callable[[T], ObservationCollectorDataClass]
