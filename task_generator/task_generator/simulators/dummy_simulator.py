from task_generator.simulators import BaseSimulator
from task_generator.shared import EntityProps, Namespace, PositionOrientation


class DummySimulator(BaseSimulator):
    """
    Does nothing.
    """

    def __init__(self, namespace: Namespace):
        super().__init__(namespace)
        self._logger = self.node.get_logger().get_child('dummysim')

    def before_reset_task(self):
        self._logger.debug("pausing")

    def after_reset_task(self):
        self._logger.debug("unpausing")

    def spawn_entity(self, entity: EntityProps) -> bool:
        self._logger.debug(f"spawning {entity.name} {repr(entity)}")
        return True

    def move_entity(self, name: str, position: PositionOrientation) -> bool:
        self._logger.debug(f"moving {name} {repr(position)}")
        return True

    def delete_entity(self, name: str) -> bool:
        self._logger.debug(f"deleting {name}")
        return True
