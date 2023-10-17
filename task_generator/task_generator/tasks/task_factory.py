from typing import Dict, Type

from task_generator.constants import Constants
from task_generator.tasks.base_task import BaseTask

class TaskFactory:
    registry: Dict[Constants.TaskMode, Type[BaseTask]] = {}

    @classmethod
    def register(cls, name: Constants.TaskMode):
        def inner_wrapper(wrapped_class):
            assert name not in cls.registry, f"TaskMode '{name}' already exists!"
            assert issubclass(wrapped_class, BaseTask)

            cls.registry[name] = wrapped_class
            return wrapped_class

        return inner_wrapper

    @classmethod
    def instantiate(cls, name: Constants.TaskMode) -> Type[BaseTask]:
        assert name in cls.registry, f"TaskMode '{name}' is not registered!"
        task = cls.registry[name]

        return task
