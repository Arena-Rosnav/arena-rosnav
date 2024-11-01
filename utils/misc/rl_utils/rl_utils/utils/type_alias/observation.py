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

from rl_utils.utils.paths import Path, PathComponent

# Gym Env
InformationDict = Dict[str, Any]

PathsDict = Dict[Type[PathComponent], Union[str, Path]]

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
