from abc import ABC, abstractmethod
from typing import Any, Dict

from task_generator.shared import Namespace


class CollectorUnit(ABC):
    _ns = Namespace
    _observation_manager = "ObservationCollector"

    def __init__(
        self, ns: Namespace, observation_manager: "ObservationCollector"
    ) -> None:
        self._ns = ns
        self._observation_manager = observation_manager

    @abstractmethod
    def init_subs(self):
        raise NotImplementedError()

    def wait(self):
        pass

    @abstractmethod
    def get_observations(
        self, obs_dict: Dict[str, Any], *args, **kwargs
    ) -> Dict[str, Any]:
        if not obs_dict:
            obs_dict = {}

        self.wait()

        return obs_dict
