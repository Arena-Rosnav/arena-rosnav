import rospy
import random

from map_distance_server.srv import GetDistanceMap, GetDistanceMapResponse
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
    skip_map = False

    # Class Methods
    def __init__(self):
        print("World Generator Initialized")
        self.map_sub = rospy.Subscriber(
            TOPIC_MAP, OccupancyGrid, self._map_callback)
        self.map_pub = rospy.Publisher(TOPIC_MAP, OccupancyGrid, queue_size=1)
        # self.static_map_srv = rospy.ServiceProxy(TOPIC_STATIC_MAP, GetMap)
        # self.new_dist_map_pub = rospy.Publisher(
        #     TOPIC_SIGNAL_DIST_MAP_NEW, String, queue_size=1
        # )
        # self.distance_map_srv = rospy.Service(
        #     "/distance_map", GetDistanceMap, self._distance_map_srv_handler
        # )

    def _map_callback(self, msg: OccupancyGrid):
        """Callback for when a new map is generated and published.

        Args:
            msg (OccupancyGrid): /map topic message.
        """
        if self.skip_map:
            self.skip_map = False
            return

        self.skip_map = True
        print(f"Skip Map Status: {self.skip_map}")
        print("Old Map Data:")
        print(msg)
        new_map: OccupancyGrid = msg
        # new_map.data = [random.choice([0,100])] * len(msg.data)
        new_map.data = [random.choice(13 * [0] + [100]) if d == 0 else d
                        for d in msg.data]
        print("New Map Data:")
        print(new_map)
        self.map_pub.publish(new_map)

    def _distance_map_srv_handler(self, _):
        msg = GetDistanceMapResponse()
        return msg


if __name__ == "__main__":
    rospy.init_node("world_generator")

    world_gen = WorldGenerator()

    while not rospy.is_shutdown():
        rospy.spin()
