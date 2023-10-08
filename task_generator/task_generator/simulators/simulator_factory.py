from typing import Any, Callable, Dict
from .base_simulator import BaseSimulator

class SimulatorFactory:
    registry: Dict[str, Callable[..., BaseSimulator]] = {}

    @classmethod
    def register(cls, name: str):
        def inner_wrapper(wrapped_class):
            assert name not in cls.registry, f"Simulator '{name}' already exists!"
            assert issubclass(wrapped_class, BaseSimulator)

            cls.registry[name] = wrapped_class
            return wrapped_class

        return inner_wrapper

    @classmethod
    def instantiate(cls, name: str) -> Callable[..., BaseSimulator]:
        assert name in cls.registry, f"Simulator '{name}' is not registered!"

        simulator = cls.registry[name]
        
        return simulator
