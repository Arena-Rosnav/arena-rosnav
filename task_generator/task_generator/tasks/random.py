import random
import time
from typing import Generator, List, Optional


from task_generator.constants import Constants
from task_generator.manager.map_manager import MapManager
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.tasks.task_factory import TaskFactory
from task_generator.shared import DynamicObstacle, Obstacle, Waypoint
from task_generator.tasks.base_task import BaseTask
from task_generator.tasks.utils import ObstacleInterface, RandomInterface, RandomList


@TaskFactory.register(Constants.TaskMode.RANDOM)
class RandomTask(BaseTask, RandomInterface):
    """
        The random task spawns static and dynamic
        obstacles on every reset and will create
        a new robot start and goal position for
        each task.
    """

    _gen_static: Generator[int, None, None]
    _gen_interactive: Generator[int, None, None]
    _gen_dynamic: Generator[int, None, None]

    static_obstacles: RandomList
    interactive_obstacles: RandomList
    dynamic_obstacles: RandomList

    def __init__(
        self,
        obstacle_manager: ObstacleManager,
        robot_managers: List[RobotManager],
        map_manager: MapManager,
        **kwargs
    ):
        BaseTask.__init__(
            self,
            obstacle_manager=obstacle_manager,
            robot_managers=robot_managers,
            map_manager=map_manager,
            **kwargs
        )

        obstacle_ranges = RandomInterface._load_obstacle_ranges(self)
        self._gen_static = RandomInterface._randrange_generator(obstacle_ranges.static)
        self._gen_interactive = RandomInterface._randrange_generator(obstacle_ranges.interactive)
        self._gen_dynamic = RandomInterface._randrange_generator(obstacle_ranges.dynamic)

        allowed_obstacles = RandomInterface._load_obstacle_list(self)
        self.static_obstacles = allowed_obstacles.static
        self.interactive_obstacles = allowed_obstacles.interactive
        self.dynamic_obstacles = allowed_obstacles.dynamic

    @BaseTask.reset_helper(parent=BaseTask)
    def reset(
        self,
        n_static_obstacles: Optional[int] = None,
        n_interactive_obstacles: Optional[int] = None, 
        n_dynamic_obstacles: Optional[int] = None,
        static_obstacles: Optional[RandomList] = None,
        interactive_obstacles: Optional[RandomList] = None,
        dynamic_obstacles: Optional[RandomList] = None,
        **kwargs):

        if n_static_obstacles is None:
            n_static_obstacles = next(self._gen_static)

        if n_interactive_obstacles is None:
            n_interactive_obstacles = next(self._gen_interactive)

        if n_dynamic_obstacles is None:
            n_dynamic_obstacles = next(self._gen_dynamic)

        if static_obstacles is None:
            static_obstacles = self.static_obstacles

        if interactive_obstacles is None:
            interactive_obstacles = self.interactive_obstacles

        if dynamic_obstacles is None:
            dynamic_obstacles = self.dynamic_obstacles

        def callback():

            self._obstacle_manager.respawn(callback=lambda: RandomInterface._setup_random(
                self,
                n_static_obstacles=n_static_obstacles,
                n_interactive_obstacles=n_interactive_obstacles,
                n_dynamic_obstacles=n_dynamic_obstacles,
                static_obstacles=static_obstacles,
                interactive_obstacles=interactive_obstacles,
                dynamic_obstacles=dynamic_obstacles
                )
            )
            time.sleep(1)
            return False

        return callback

    