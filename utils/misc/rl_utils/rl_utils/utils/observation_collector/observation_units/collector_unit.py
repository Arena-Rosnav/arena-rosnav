from abc import ABC, abstractmethod
from typing import Any, Dict

from task_generator.shared import Namespace


class CollectorUnit(ABC):
    """
    Abstract base class for defining a collector unit.

    Attributes:
    - _ns (Namespace): The namespace for the collector unit.
    - _observation_manager (ObservationManager): The observation manager containing the unit.

    Methods:
    - __init__: Initializes the collector unit with the provided namespace and observation manager.
    - init_subs: Abstract method to initialize subscriptions.
    - wait: Method to wait for observations.
    - get_observations: Abstract method to process the observations and updating the observation dict.
    """

    _ns = Namespace
    _observation_manager = "ObservationCollector"

    def __init__(
        self,
        ns: Namespace,
        observation_manager: "ObservationCollector",
        *args,
        **kwargs
    ) -> None:
        """
        Initializes the collector unit with the provided namespace and observation manager.

        Args:
        - ns (Namespace): The namespace for the collector unit.
        - observation_manager (ObservationCollector): The observation manager for the collector unit.
        """
        self._ns = ns
        self._observation_manager = observation_manager

    @abstractmethod
    def init_subs(self):
        """
        Abstract method to initialize subscriptions.
        """
        raise NotImplementedError()

    def wait(self):
        """
        Method to wait for observations.
        """
        pass

    @abstractmethod
    def get_observations(
        self, obs_dict: Dict[str, Any], *args, **kwargs
    ) -> Dict[str, Any]:
        """
        Abstract method to process the observations and updating the observation dict.

        Args:
        - obs_dict (Dict[str, Any]): Dictionary to store observations.

        Returns:
        - Dict[str, Any]: Updated dictionary containing the collected observations.
        """
        if not obs_dict:
            obs_dict = {}

        self.wait()

        return obs_dict
