from .base_environment import BaseEnvironment

class EnvironmentFactory:
    registry = {}

    @classmethod
    def register(cls, name):
        def inner_wrapper(wrapped_class):
            assert name not in cls.registry, f"Environment '{name}' already exists!"
            assert issubclass(wrapped_class, BaseEnvironment)

            cls.registry[name] = wrapped_class
            return wrapped_class

        return inner_wrapper

    @classmethod
    def instantiate(cls, name: str):
        assert name in cls.registry, f"Environment '{name}' is not registered!"

        environment = cls.registry[name]
        
        return environment
