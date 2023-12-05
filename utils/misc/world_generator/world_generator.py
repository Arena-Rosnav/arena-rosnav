import rospy
import roslaunch

from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from std_msgs.msg import String


# TOPICS/SERVICES
TOPIC_MAP = "/map"
TOPIC_MAP_NEW = "/world/map"
TOPIC_STATIC_MAP = "/static_map"
TOPIC_STATIC_MAP_NEW = "/world/static_map"
TOPIC_SIGNAL_DIST_MAP = "/signal_new_distance_map"
TOPIC_SIGNAL_DIST_MAP_NEW = "/world/signal_new_distance_map"
SERVICE_DISTANCE_MAP = "/distance_map"
SERVICE_DISTANCE_MAP_NEW = "/world/distance_map"


class WorldGenerator:
    """
    This Class handles the maps generated in the Map Generator 
    and transforms them into a more realistic world.
    """
    # Class Properties

    # Class Methods
    def __init__(self):

        rospy.loginfo(
            "-=-=-=-=-=-==-=-=-=-=-==-=-=-==-=-=-=-=-==-=-=-=-=-=====-=-=-=-=-==-=-=-=-=")
        rospy.loginfo(
            "-=-=-=-=-=-==-=-=-=-=-==-=-=-==-=-=-=-=-==-=-=-=-=-=====-=-=-=-=-==-=-=-=-=")
        rospy.loginfo(
            "-=-=-=-=-=-==-=-=-=-=-==-=-=-==-=-=-=-=-==-=-=-=-=-=====-=-=-=-=-==-=-=-=-=")
        self.map_sub = rospy.Subscriber(
            TOPIC_MAP, OccupancyGrid, self._map_callback)
        self.static_map_srv = rospy.ServiceProxy(TOPIC_STATIC_MAP, GetMap)
        self.new_dist_map_pub = rospy.Publisher(
            TOPIC_SIGNAL_DIST_MAP, String, queue_size=1
        )

    def _map_callback(self, msg: OccupancyGrid):
        """Callback for when a new map is generated and published.

        Args:
            msg (OccupancyGrid): /map topic message.
        """
        rospy.loginfo("----------============---------------------")
        rospy.loginfo(msg)
        rospy.loginfo("-------------------------------------------")


def start():
    rospy.init_node("world_generator_node", anonymous=False)

    world_gen = WorldGenerator()

    pkg = "world_generator"
    executable = "world_generator.py"
    remap_args = (
        None
        if rospy.get_param("single_env", True)
        else [("/clock", "/clock_simulation")]
    )

    node = roslaunch.core.Node(
        pkg,
        executable,
        remap_args=remap_args,
    )

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    process = launch.launch(node)

    while not rospy.is_shutdown():
        rospy.spin()

    process.stop()


if __name__ == "__main__":
    start()
