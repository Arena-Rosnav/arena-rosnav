from typing import Dict, TypeVar, Union, Generic, Callable

from .collectors import *
from .generators import *
from .static import *


# Type Variables
ObservationGenerator = TypeVar("ObservationGenerator", bound=ObservationGeneratorUnit)
ObservationCollector = TypeVar("ObservationCollector", bound=ObservationCollectorUnit)
TypeObservationGeneric = Union[ObservationCollector, ObservationGenerator]

ObservationGeneric = Union[ObservationCollectorUnit, ObservationGeneratorUnit]

ObservationGenericName = str
ObservationGenericDataClass = Union[
    ObservationCollectorUnit.data_class, ObservationGeneratorUnit.data_class
]

ObservationDict = Dict[ObservationGenericName, ObservationGenericDataClass]

ObservationCollectorDataClass = TypeVar("ObservationCollectorDataClass")

T = TypeVar("T")
ProcessingFunction = Callable[[T], ObservationCollectorDataClass]


class GenericObservation(Generic[ObservationCollectorDataClass]):
    """
    Represents a generic observation.

    This class is responsible for holding and processing observations.
    It provides methods to update the observation, check if it is stale,
    and invalidate the observation.

    Attributes:
        _value (ObservationCollectorDataClass): The processed observation value.
        _stale (bool): Indicates whether the observation is stale or not.
    """

    _value: ObservationCollectorDataClass
    _stale: bool

    def __init__(
        self, initial_msg: T, process_fnc: ProcessingFunction = lambda x: x
    ) -> None:
        """
        Initializes a new instance of the GenericObservation class.

        Args:
            initial_msg (T): The initial observation message.
            process_fnc (ProcessingFnc, optional): The function used to process the observation.
                Defaults to lambda x: x.
        """
        self._msg = initial_msg
        self._process_fnc = process_fnc

        self._value = process_fnc(initial_msg)
        self._stale = True

    @property
    def stale(self) -> bool:
        """
        Gets or sets the stale flag of the observation.

        Returns:
            bool: True if the observation is stale, False otherwise.
        """
        return self._stale

    @stale.setter
    def stale(self, value: bool):
        """
        Sets the stale flag of the observation.

        Args:
            value (bool): The value to set for the stale flag.
        """
        self._stale = value

    @property
    def value(self) -> ObservationCollectorDataClass:
        """
        Gets the processed observation value.

        Returns:
            ObservationCollectorDataClass: The processed observation value.
        """
        return self._value

    def update(self, msg: T):
        """
        Updates the observation with a new message.

        Args:
            msg (T): The new observation message.
        """
        self._msg = msg
        self._value = self._process_fnc(msg)
        self._stale = False

    def invalidate(self):
        """
        Invalidates the observation by setting the stale flag to True.
        """
        self._stale = True
