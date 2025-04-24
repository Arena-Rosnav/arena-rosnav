from task_generator.manager.entity_manager import EntityManager
from task_generator.shared import DynamicObstacle, Obstacle


class DummyEntityManager(EntityManager):

    _logger_name = 'dummy_EM'

    def _spawn_obstacle_impl(
        self,
        obstacle,
    ) -> Obstacle:
        return obstacle

    def _spawn_dynamic_obstacle_impl(
        self,
        obstacle,
    ) -> DynamicObstacle:
        return obstacle

    def _remove_obstacles_impl(
        self,
    ) -> bool:
        return True

    def _spawn_walls_impl(
        self,
        walls,
    ) -> bool:
        return True

    def _spawn_robot_impl(
        self,
        robot,
    ) -> bool:
        return True

    def _remove_robot_impl(
        self,
        name,
    ) -> bool:
        return True

    def _move_robot_impl(
        self,
        name,
        position,
    ) -> bool:
        return True
