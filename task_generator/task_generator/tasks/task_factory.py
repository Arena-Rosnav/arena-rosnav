from .base_task import BaseTask

class TaskFactory:
    registry = {}

    @classmethod
    def register(cls, name):
        def inner_wrapper(wrapped_class):
            assert name not in cls.registry, f"Simulator '{name}' already exists!"
            assert issubclass(wrapped_class, BaseTask)

            cls.registry[name] = wrapped_class
            return wrapped_class

        return inner_wrapper

    @classmethod
    def instantiate(cls, name: str, *args, **kwargs):
        assert name in cls.registry, f"Simulator '{name}' is not registered!"
        simulator = cls.registry[name]
        
        if issubclass(simulator, BaseTask):
            return simulator(*args, **kwargs)
        else:
            return simulator
