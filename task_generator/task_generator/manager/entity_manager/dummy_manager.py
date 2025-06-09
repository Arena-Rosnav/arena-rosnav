from task_generator.manager.entity_manager import EntityManager
from task_generator.shared import DynamicObstacle, Obstacle


class DummyEntityManager(EntityManager):

    def _spawn_obstacle_impl(
        self,
        obstacle,
    ) -> Obstacle | None:
        return obstacle

    def _spawn_dynamic_obstacle_impl(
        self,
        obstacle,
    ) -> DynamicObstacle | None:
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
        pose,
    ) -> bool:
        return True
