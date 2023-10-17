from typing import List
import rospy

from nav_msgs.msg import OccupancyGrid
from map_distance_server.srv import GetDistanceMap
from std_msgs.msg import String

from task_generator.constants import Constants
from task_generator.manager.map_manager import MapManager
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.tasks.random import RandomTask
from task_generator.tasks.task_factory import TaskFactory


@TaskFactory.register(Constants.TaskMode.DYNAMIC_MAP_RANDOM)
class DynamicMapRandomTask(RandomTask):
    """
    The random task spawns static and dynamic
    obstacles on every reset and will create
    a new robot start and goal position for
    each task.
    """

    _eps_per_map: float
    _iterator: float

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

        # requests new map from map generator
        self.map_request_pub = rospy.Publisher(
            "/request_new_map", String, queue_size=1)
        # requests distance map from distance map server
        self.get_dist_map_service = rospy.ServiceProxy(
            "/distance_map", GetDistanceMap)

        # task reset for all taskmanagers when one resets
        self.task_reset_pub = rospy.Publisher(
            "/dynamic_map/task_reset", String, queue_size=1
        )
        self.task_reset_sub = rospy.Subscriber(
            "/dynamic_map/task_reset", String, self._cb_task_reset
        )

        # iterate resets over 1 / num_envs
        # e.g. eps_per_map = 2
        # if num_envs = 2, then 1 / 2 = 0.5
        # in sum we need 4 resets to get to the next map -> 4 * 0.5 = 2
        # eps_per_map = sum_resets * 1 / num_envs
        self._eps_per_map = float(str(rospy.get_param("episode_per_map", 1)))
        denominator: float = (
            float(str(rospy.get_param("num_envs", 1)))
            if self._robot_managers[0].namespace != "eval_sim"
            else 1
        )
        self._iterator = 1 / denominator

        rospy.set_param("/dynamic_map/curr_eps", 0)

    def update_map(self):
        self._map_manager.update_map(self.get_dist_map_service())

    def reset(
        self,
        reset_after_new_map: bool = False,
        first_map: bool = False,
        **kwargs
    ):
        def callback() -> bool:
            try:
                if (
                    float(str(rospy.get_param("/dynamic_map/curr_eps", 0))
                        ) >= self._eps_per_map
                    or first_map
                ):
                    self.request_new_map(first_map=first_map)
                    return False
                
            except Exception:
                pass

            if not reset_after_new_map:
                # only update eps count when resetting the scene
                new_count = float(str(rospy.get_param("/dynamic_map/curr_eps")))
                new_count += self._iterator
                rospy.set_param("/dynamic_map/curr_eps", new_count)

            return False

        return super().reset(callback=callback, **kwargs)

    def request_new_map(self, first_map: bool = False):
        # set current eps immediately to 0 so that only one task
        # requests a new map
        if not first_map:
            rospy.set_param("/dynamic_map/curr_eps", 0)

        self.map_request_pub.publish("")

        rospy.wait_for_message("/map", OccupancyGrid)
        rospy.wait_for_message("/signal_new_distance_map", String)

        self.task_reset_pub.publish("")
        self.update_map()

        rospy.loginfo("===================")
        rospy.loginfo("+++ Got new map +++")
        rospy.loginfo("===================")

    def _cb_task_reset(self, msg):
        # task reset for all taskmanagers when one resets
        # update map manager
        self.update_map()
        self.reset(reset_after_new_map=True)
