from typing import List
import rospy

from nav_msgs.msg import OccupancyGrid
from map_distance_server.srv import GetDistanceMap
from std_msgs.msg import String

from task_generator.constants import Constants
from task_generator.manager.map_manager import MapManager
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.tasks.base_task import BaseTask
from task_generator.tasks.random import RandomTask
from task_generator.tasks.task_factory import TaskFactory
from task_generator.tasks.utils import DynamicMapInterface, RandomInterface
from task_generator.utils import rosparam_get


@TaskFactory.register(Constants.TaskMode.DYNAMIC_MAP_RANDOM)
class DynamicMapRandomTask(BaseTask, DynamicMapInterface):
    """
    The random task spawns static and dynamic
    obstacles on every reset and will create
    a new robot start and goal position for
    each task.
    """

    _eps_per_map: float
    _iterator: float

    map_request_pub: rospy.Publisher
    get_dist_map_service: rospy.ServiceProxy
    task_reset_pub: rospy.Publisher

    def __init__(
        self,
        obstacle_manager: ObstacleManager,
        robot_managers: List[RobotManager],
        map_manager: MapManager,
        **kwargs
    ):
        super().__init__(obstacle_manager=obstacle_manager,
                         robot_managers=robot_managers, map_manager=map_manager, **kwargs)
        if rospy.get_param("map_file") != Constants.MapGenerator.MAP_FOLDER_NAME:
            raise ValueError(
                "'DYNAMIC_MAP_RANDOM' task can only be used with dynamic map, "
                "otherwise the MapGenerator isn't used."
            )

        DynamicMapInterface.subscribe(self, self._cb_task_reset)
        self.map_request_pub, self.task_reset_pub = DynamicMapInterface.create_publishers(self)
        self.get_dist_map_service = DynamicMapInterface.get_service(self)

        # iterate resets over 1 / num_envs
        # e.g. eps_per_map = 2
        # if num_envs = 2, then 1 / 2 = 0.5
        # in sum we need 4 resets to get to the next map -> 4 * 0.5 = 2
        # eps_per_map = sum_resets * 1 / num_envs
        self._eps_per_map = float(str(rospy.get_param("episode_per_map", 1)))
        denominator: float = (
            rosparam_get(float, "num_envs", 1)
            if self._robot_managers[0].namespace != "eval_sim"
            else 1
        )
        self._iterator = 1 / denominator

        rospy.set_param("/dynamic_map/curr_eps", 0)

    
    @BaseTask.reset_helper(parent=BaseTask)
    def reset(
        self,
        reset_after_new_map: bool = False,
        first_map: bool = False,
        **kwargs
    ):
        
        if first_map is None:
            first_map = rosparam_get(float, "/dynamic_map/curr_eps", 0) >= self._eps_per_map

        def callback() -> bool:
            try:
                self.request_new_map(first_map=first_map, request_pub=self.map_request_pub, reset_pub=self.task_reset_pub)
                DynamicMapInterface.update_map(self, dist_map=self.get_dist_map_service())
                return False
                
            except Exception:
                pass

            if not reset_after_new_map:
                # only update eps count when resetting the scene
                DynamicMapInterface.increment_episodes(self, step=self._iterator)

            return False

        return callback

    

    def _cb_task_reset(self, msg):
        # task reset for all taskmanagers when one resets
        # update map manager
        self.update_map(dist_map=self.get_dist_map_service())
        self.reset(reset_after_new_map=True)
