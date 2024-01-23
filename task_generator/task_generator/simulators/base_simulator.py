import itertools
from typing import Callable, Collection, Dict

from task_generator.shared import ModelType, EntityProps, Namespace, PositionOrientation


class BaseSimulator:

    _namespace: Namespace

    _spawn_model: Dict[ModelType, Callable]

    __counter: itertools.count

    def __init__(self, namespace: Namespace):
        self._namespace = namespace
        self._spawn_model = dict()

        self.__counter = itertools.count()

    def generate_random_name(self) -> str:
        return f"random_name_{next(self.__counter)}"

    @property
    def MODEL_TYPES(self) -> Collection[ModelType]:
        return self._spawn_model.keys()

    def spawn_model(self, model_type: ModelType, *args, **kwargs):
        if model_type in self._spawn_model:
            return self._spawn_model[model_type](*args, **kwargs)

        raise NotImplementedError(
            f"{type(self).__name__} does not implement spawn_model[{model_type}]")

    def before_reset_task(self):
        """
        Is executed each time before the task is reset. This is useful in
        order to pause the simulation.
        """
        raise NotImplementedError()

    def after_reset_task(self):
        """
        Is executed after the task is reset. This is useful to unpause the
        simulation.
        """
        raise NotImplementedError()

    def spawn_entity(self, entity: EntityProps) -> bool:
        raise NotImplementedError()

    def move_entity(self, name: str, position: PositionOrientation) -> bool:
        """
        Move the robot to the given position.
        """
        raise NotImplementedError()

    def delete_entity(self, name: str) -> bool:
        raise NotImplementedError()
