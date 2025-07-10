from arena_rclpy_mixins.shared import Namespace

from task_generator.shared import Entity, Pose, Wall
from task_generator.simulators.sim import BaseSim


class DummySimulator(BaseSim):
    """
    Does nothing.
    """

    def __init__(self, namespace: Namespace):
        super().__init__(namespace)

    def before_reset_task(self):
        self._logger.debug("pausing")

    def after_reset_task(self):
        self._logger.debug("unpausing")

    def spawn_entity(self, entity: Entity) -> bool:
        self._logger.debug(f"spawning {entity.name} {repr(entity)}")
        return True

    def move_entity(self, name: str, pose: Pose) -> bool:
        self._logger.debug(f"moving {name} {repr(pose)}")
        return True

    def delete_entity(self, name: str) -> bool:
        self._logger.debug(f"deleting {name}")
        return True

    def spawn_walls(self, walls: list[Wall]) -> bool:
        self._logger.debug(f'spawning {len(walls)} walls')
        return True
