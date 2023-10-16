from typing import Dict, Type

from task_generator.constants import Constants
from task_generator.simulators.base_simulator import BaseSimulator


class SimulatorFactory:
    registry: Dict[Constants.Simulator, Type[BaseSimulator]] = {}

    @classmethod
    def register(cls, name: Constants.Simulator):
        def inner_wrapper(wrapped_class):
            assert name not in cls.registry, f"Simulator '{name}' already exists!"
            assert issubclass(wrapped_class, BaseSimulator)

            cls.registry[name] = wrapped_class
            return wrapped_class

        return inner_wrapper

    @classmethod
    def instantiate(cls, name: Constants.Simulator) -> Type[BaseSimulator]:
        assert name in cls.registry, f"Simulator '{name}' is not registered!"

        simulator = cls.registry[name]

        return simulator
