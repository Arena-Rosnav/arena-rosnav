from typing import (
    Any,
    Callable,
    Dict,
    List,
    Optional,
    Tuple,
    Type,
    TypeVar,
    Union,
    Literal,
    TYPE_CHECKING,
    NewType,
)


PATHS_KEYS = Literal["model", "tb", "eval", "robot_setting", "config", "curriculum"]
PathsDict = Dict[PATHS_KEYS, str]

CustomDiscreteAction = Dict[str, Union[str, float]]
CustomDiscreteActionList = List[CustomDiscreteAction]

# Observation Collector
from rl_utils.utils.observation_collector.collectors.base_collector import (
    ObservationCollectorUnit,
)
from rl_utils.utils.observation_collector.generators.base_generator import (
    ObservationGeneratorUnit,
)

ObservationGenerator = TypeVar("ObservationGenerator", bound=ObservationGeneratorUnit)
ObservationCollector = TypeVar("ObservationCollector", bound=ObservationCollectorUnit)
TypeObservationGeneric = Union[ObservationCollector, ObservationGenerator]

ObservationGeneric = Union[ObservationCollectorUnit, ObservationGeneratorUnit]

ObservationGenericName = NewType("ObservationGenericName", str)
ObservationGenericDataClass = Union[
    ObservationCollectorUnit.data_class, ObservationGeneratorUnit.data_class
]

ObservationDict = Dict[ObservationGenericName, ObservationGenericDataClass]

ObservationCollectorDataClass = TypeVar("ObservationCollectorDataClass")

T = TypeVar("T")
ProcessingFunction = Callable[[T], ObservationCollectorDataClass]
