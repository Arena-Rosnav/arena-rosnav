from .base_map_gen import BaseMapGenerator


class MapGeneratorFactory:
    registry = {}

    @classmethod
    def register(cls, name):
        def inner_wrapper(wrapped_class):
            assert name not in cls.registry, f"Generator '{name}' already exists!"
            assert issubclass(wrapped_class, BaseMapGenerator)

            cls.registry[name] = wrapped_class
            return wrapped_class

        return inner_wrapper

    @classmethod
    def instantiate(cls, name: str, *args, **kwargs):
        assert name in cls.registry, f"Generator '{name}' is not registered!"
        simulator = cls.registry[name]

        return simulator(*args, **kwargs)
