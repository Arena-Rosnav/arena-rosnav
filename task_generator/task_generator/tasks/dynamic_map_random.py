from task_generator.constants import Constants
from task_generator.tasks.base_task import BaseTask
from task_generator.tasks.random import RandomTask
from task_generator.tasks.task_factory import TaskFactory
from task_generator.tasks.utils import ITF_DynamicMap
from task_generator.utils import rosparam_get


@TaskFactory.register(Constants.TaskMode.DYNAMIC_MAP_RANDOM)
class DynamicMapRandomTask(RandomTask):
    """
    The random task spawns static and dynamic
    obstacles on every reset and will create
    a new robot start and goal position for
    each task.
    """

    itf_dynamicmap: ITF_DynamicMap

    _eps_per_map: float
    _iterator: float

    def __init__(self, **kwargs):
        RandomTask.__init__(self, **kwargs)

        self.itf_dynamicmap = ITF_DynamicMap(
            self, configurations=ITF_DynamicMap.const_config({})  # TODO
        )
        self.itf_dynamicmap.subscribe_reset(callback=self._cb_task_reset)

        # iterate resets over 1 / num_envs
        # e.g. eps_per_map = 2
        # if num_envs = 2, then 1 / 2 = 0.5
        # in sum we need 4 resets to get to the next map -> 4 * 0.5 = 2
        # eps_per_map = sum_resets * 1 / num_envs
        self._eps_per_map = rosparam_get(float, "episode_per_map", 1.0)
        denominator: float = (
            rosparam_get(float, "num_envs", 1.0)
            if "eval_sim" not in self.robot_managers[0].namespace
            else 1.0
        )
        self._iterator = 1 / denominator

        self.itf_dynamicmap.episodes = 0

    @BaseTask.reset_helper(parent=RandomTask)
    def reset(
        self, reset_after_new_map: bool = False, first_map: bool = False, **kwargs
    ):
        if first_map or self.itf_dynamicmap.episodes >= self._eps_per_map:
            self.itf_dynamicmap.request_new_map(first_map=first_map)

            return {}, None

        if not reset_after_new_map:
            # only update eps count when resetting the scene
            self.itf_dynamicmap.episodes += self._iterator

        def callback():
            return False

        return {}, callback

    def _cb_task_reset(self, *args, **kwargs):
        # task reset for all taskmanagers when one resets
        # update map manager
        self.itf_dynamicmap.update_map()
        self.reset(callback=lambda: None, reset_after_new_map=True)
