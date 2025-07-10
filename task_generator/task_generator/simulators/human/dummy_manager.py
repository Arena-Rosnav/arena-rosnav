from collections.abc import Sequence
from task_generator.simulators.human import BaseHumanSimulator
from task_generator.shared import DynamicObstacle, Obstacle


class DummyHumanSimulator(BaseHumanSimulator):

    def _spawn_obstacle_impl(
        self,
        obstacles,
    ) -> Sequence[Obstacle | None]:
        return obstacles

    def _spawn_dynamic_obstacle_impl(
        self,
        obstacles,
    ) -> Sequence[DynamicObstacle | None]:
        return obstacles

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
