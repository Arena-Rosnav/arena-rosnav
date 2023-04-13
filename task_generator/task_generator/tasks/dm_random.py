import rospy

from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from map_distance_server.srv import GetDistanceMap
from std_msgs.msg import String

from task_generator.constants import Constants, TaskMode
from task_generator.tasks.task_factory import TaskFactory
from .random import RandomTask
from ..manager.obstacle_manager import ObstacleManager
from ..manager.map_manager import MapManager
from ..manager.robot_manager import RobotManager


@TaskFactory.register(TaskMode.DM_RANDOM)
class DynamicMapRandomTask(RandomTask):
    """
    The random task spawns static and dynamic
    obstacles on every reset and will create
    a new robot start and goal position for
    each task.
    """

    def __init__(
        self,
        obstacles_manager: ObstacleManager,
        robot_managers: RobotManager,
        map_manager: MapManager,
        *args,
        **kwargs
    ):
        super().__init__(
            obstacles_manager, robot_managers, map_manager, *args, **kwargs
        )
        if rospy.get_param("map_file") != Constants.MapGenerator.MAP_FOLDER_NAME:
            raise ValueError(
                "'DYNAMIC_MAP_RANDOM' task can only be used with dynamic map, "
                "otherwise the MapGenerator isn't used."
            )

        # requests new map from map generator
        self.map_request_pub = rospy.Publisher("/request_new_map", String, queue_size=1)
        # requests distance map from distance map server
        self.get_dist_map_service = rospy.ServiceProxy("/distance_map", GetDistanceMap)

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
        self.eps_per_map = rospy.get_param("episode_per_map", 1)
        denominator = (
            rospy.get_param("num_envs", 1)
            if self.robot_managers[0].namespace != "eval_sim"
            else 1
        )
        self.iterator = 1 / denominator

        rospy.set_param("/dynamic_map/curr_eps", 0)

    def update_map(self):
        self.map_manager.update_map(self.get_dist_map_service())

    def reset(
        self,
        start=None,
        goal=None,
        static_obstacles=None,
        dynamic_obstacles=None,
        reset_after_new_map=False,
        first_map=False,
    ):
        if rospy.get_param("/dynamic_map/curr_eps") >= self.eps_per_map or first_map:
            self.request_new_map(first_map=first_map)
            return

        if not reset_after_new_map:
            # only update eps count when resetting the scene
            new_count = rospy.get_param("/dynamic_map/curr_eps") + self.iterator
            rospy.set_param("/dynamic_map/curr_eps", new_count)

        return super().reset(
            start=start,
            goal=goal,
            static_obstacles=static_obstacles,
            dynamic_obstacles=dynamic_obstacles,
        )

    def request_new_map(self, first_map: bool = False):
        # set current eps immediately to 0 so that only one task
        # requests a new map
        if not first_map:
            rospy.set_param("/dynamic_map/curr_eps", 0)

        self.map_request_pub.publish("")

        rospy.wait_for_message("/map", OccupancyGrid)
        rospy.wait_for_message("/signal_new_distance_map", String)

        self.task_reset_pub.publish("")

        rospy.loginfo("===================")
        rospy.loginfo("+++ Got new map +++")
        rospy.loginfo("===================")

    def _cb_task_reset(self, msg):
        # task reset for all taskmanagers when one resets
        # update map manager
        self.update_map()
        self.reset(reset_after_new_map=True)
