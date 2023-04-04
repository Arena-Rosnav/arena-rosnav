import random
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


@TaskFactory.register(TaskMode.DYNAMIC_MAP_RANDOM)
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

        rospy.wait_for_service("/static_map")

        self.map_request_pub = rospy.Publisher("/request_new_map", String, queue_size=1)
        self.get_dist_map_service = rospy.ServiceProxy("/distance_map", GetDistanceMap)

        self.eps_per_map = Constants.MapGenerator.EPS_PER_MAP
        self.curr_map_eps_count = 0

    def update_map(self):
        self.map_manager.update_map(self.get_dist_map_service())

    def reset(
        self,
        start=None,
        goal=None,
        static_obstacles=None,
        dynamic_obstacles=None,
    ):
        if self.curr_map_eps_count >= self.eps_per_map:
            rospy.loginfo("=============")
            rospy.loginfo("Requesting new map")

            self.map_request_pub.publish("")
            rospy.wait_for_message("/map", OccupancyGrid)
            rospy.wait_for_message("/signal_new_distance_map", String)
            # update map manager
            self.update_map()

            rospy.loginfo("Got new map")
            rospy.loginfo("=============")

            self.curr_map_eps_count = 0

        self.curr_map_eps_count += 1
        return super().reset(
            start=start,
            goal=goal,
            static_obstacles=static_obstacles,
            dynamic_obstacles=dynamic_obstacles,
        )
