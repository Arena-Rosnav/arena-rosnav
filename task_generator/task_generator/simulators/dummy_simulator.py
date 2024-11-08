from task_generator.simulators import BaseSimulator
from task_generator.shared import EntityProps, PositionOrientation
from task_generator import TASKGEN_NODE

_logger = TASKGEN_NODE.get_logger().get_child('dummysim')


class DummySimulator(BaseSimulator):
    """
    Does nothing.
    """

    def before_reset_task(self):
        _logger.debug("pausing")

    def after_reset_task(self):
        _logger.debug("unpausing")

    def spawn_entity(self, entity: EntityProps) -> bool:
        _logger.debug(f"spawning {entity.name} {repr(entity)}")
        return True

    def move_entity(self, name: str, position: PositionOrientation) -> bool:
        _logger.debug(f"moving {name} {repr(position)}")
        return True

    def delete_entity(self, name: str) -> bool:
        _logger.debug(f"deleting {name}")
        return True
