import time
from typing import Generator, Optional


from task_generator.constants import Constants
from task_generator.tasks.task_factory import TaskFactory
from task_generator.tasks.base_task import BaseTask
from task_generator.tasks.utils import ITF_Random, RandomList


@TaskFactory.register(Constants.TaskMode.RANDOM)
class RandomTask(BaseTask):
    """
    The random task spawns static and dynamic
    obstacles on every reset and will create
    a new robot start and goal position for
    each task.
    """

    itf_random: ITF_Random

    _gen_static: Generator[int, None, None]
    _gen_interactive: Generator[int, None, None]
    _gen_dynamic: Generator[int, None, None]

    _static_obstacles: RandomList
    _interactive_obstacles: RandomList
    _dynamic_obstacles: RandomList

    def __init__(self, **kwargs):
        BaseTask.__init__(self, **kwargs)

        self.itf_random = ITF_Random(self)

        obstacle_ranges = self.itf_random.load_obstacle_ranges()
        self._gen_static = ITF_Random.randrange_generator(obstacle_ranges.static)
        self._gen_interactive = ITF_Random.randrange_generator(
            obstacle_ranges.interactive
        )
        self._gen_dynamic = ITF_Random.randrange_generator(obstacle_ranges.dynamic)

        (
            self._static_obstacles,
            self._interactive_obstacles,
            self._dynamic_obstacles,
        ) = self.itf_random.load_obstacle_list()

        self.iters = 0

    @BaseTask.reset_helper(parent=BaseTask)
    def reset(
        self,
        n_static_obstacles: Optional[int] = None,
        n_interactive_obstacles: Optional[int] = None,
        n_dynamic_obstacles: Optional[int] = None,
        static_obstacles: Optional[RandomList] = None,
        interactive_obstacles: Optional[RandomList] = None,
        dynamic_obstacles: Optional[RandomList] = None,
        **kwargs
    ):
        if n_static_obstacles is None:
            n_static_obstacles = next(self._gen_static)

        if n_interactive_obstacles is None:
            n_interactive_obstacles = next(self._gen_interactive)

        if n_dynamic_obstacles is None:
            n_dynamic_obstacles = next(self._gen_dynamic)

        if static_obstacles is None:
            static_obstacles = self._static_obstacles

        if interactive_obstacles is None:
            interactive_obstacles = self._interactive_obstacles

        if dynamic_obstacles is None:
            dynamic_obstacles = self._dynamic_obstacles

        def callback():
            self.obstacle_manager.respawn(
                callback=lambda: self.itf_random.setup_random(
                    n_static_obstacles=n_static_obstacles,
                    n_interactive_obstacles=n_interactive_obstacles,
                    n_dynamic_obstacles=n_dynamic_obstacles,
                    static_obstacles=static_obstacles,
                    interactive_obstacles=interactive_obstacles,
                    dynamic_obstacles=dynamic_obstacles,
                )
            )
            self.iters += 1

            return False

        return {}, callback
