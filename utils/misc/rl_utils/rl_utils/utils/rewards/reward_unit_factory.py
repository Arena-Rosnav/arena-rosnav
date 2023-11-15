from typing import Dict, Type

from .base_reward_units import RewardUnit


class RewardUnitFactory:
    registry: Dict[str, Type[RewardUnit]] = {}

    @classmethod
    def register(cls, name: str):
        def inner_wrapper(wrapped_class: RewardUnit):
            assert name not in cls.registry, f"RewardUnit '{name}' already exists!"
            assert issubclass(wrapped_class, RewardUnit)

            cls.registry[name] = wrapped_class
            return wrapped_class

        return inner_wrapper

    @classmethod
    def instantiate(cls, name: str) -> Type[RewardUnit]:
        assert name in cls.registry, f"RewardUnit '{name}' is not registered!"
        return cls.registry[name]
