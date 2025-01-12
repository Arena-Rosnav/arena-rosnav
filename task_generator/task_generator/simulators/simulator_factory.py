from typing import Dict, Type

from rl_utils.utils.constants import Simulator

from task_generator.simulators.base_simulator import BaseSimulator


class SimulatorFactory:
    registry: Dict[Simulator, Type[BaseSimulator]] = {}

    @classmethod
    def register(cls, name: Simulator):
        def inner_wrapper(wrapped_class):
            assert name not in cls.registry, f"Simulator '{name}' already exists!"
            assert issubclass(wrapped_class, BaseSimulator)

            cls.registry[name] = wrapped_class
            return wrapped_class

        return inner_wrapper

    @classmethod
    def instantiate(cls, name: Simulator) -> Type[BaseSimulator]:
        assert name in cls.registry, f"Simulator '{name}' is not registered!"

        simulator = cls.registry[name]

        return simulator
