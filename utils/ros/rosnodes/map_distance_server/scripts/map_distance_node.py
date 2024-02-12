#!/usr/bin/env python3
import os
from math import isclose
from pathlib import Path

import numpy as np
import rospkg
import rospy
from map_distance_server.srv import GetDistanceMap, GetDistanceMapResponse
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from PIL import Image
from std_msgs.msg import String

from map_generator.constants import MAP_GENERATOR_NS


def nearlyequal(a, b, sigfig=5):
    return round(abs(a - b), sigfig) == 0


def print_map(map):
    for row in map:
        print(list(row))


class MapDistanceServer:
    def __init__(self):
        self._distance_map_path = os.path.join(
            Path(rospkg.RosPack().get_path("arena_simulation_setup")),
            "worlds",
            rospy.get_param("map_file"),
            "map",
            "distance_map.png",
        )

        rospy.wait_for_service("/static_map")
        self.map_service = rospy.ServiceProxy("/static_map", GetMap)

        self.produce_distance_map()

        self.distance_map_srv = rospy.Service(
            "/distance_map", GetDistanceMap, self._distance_map_srv_handler
        )

    def produce_distance_map(self):
        """Generates and saves or loads the distance map."""
        self.map = self.map_service().map

        if not os.path.exists(self._distance_map_path):
            # If the distance map does not exist or a new map is provided by map generator
            distance_map = None
            self.new_map_data = list(self._get_map_with_distances())
            self.save_distance_map(
                self._distance_map_path, self.new_map_data, self.map.info
            )
        else:
            distance_map = self.get_distance_map(self._distance_map_path, self.map.info)
            self.new_map_data = distance_map

    def _distance_map_srv_handler(self, _):
        msg = GetDistanceMapResponse()

        msg.header = self.map.header
        msg.info = self.map.info

        msg.data = self.new_map_data

        return msg

    def create_distance_color(self, value):
        if value < 0:
            return (0, 0, 0)

        as_bits = "{0:b}".format(pow(255, 3) - value)

        return tuple(int(as_bits[(i * 8) : ((i + 1) * 8)], base=2) for i in range(3))

    def create_distance_value(self, color):
        m = int("".join(["{0:b}".format(i).rjust(8, "0") for i in color]), base=2)

        return 0 if m == 0 else pow(255, 3) - m

    def save_distance_map(self, path: str, distance_map, info):
        width = info.width
        height = info.height

        distance_map_img = np.array(
            list(map(self.create_distance_color, distance_map))
        ).ravel()

        distance_map_img = np.reshape(distance_map_img, (height, width, 3))

        image = Image.fromarray(np.uint8(distance_map_img), "RGB")

        image.save(path)

    def get_distance_map(self, distance_map_path, info):
        image = map(
            self.create_distance_value,
            np.array(Image.open(distance_map_path))
            .ravel()
            .reshape((info.width * info.height, 3)),
        )

        return list(image)

    def _get_map_with_distances(self):
        width_in_cell, height_in_cell = self.map.info.width, self.map.info.height

        map_2d = np.reshape(self.map.data, (height_in_cell, width_in_cell))

        free_space_indices = np.where(map_2d == 0)
        free_space_coordinates = np.array(free_space_indices).transpose()

        coordinates_with_length = np.full(
            len(self.map.data), -1
        )  ## Hier wird die Länge gespeichert
        coordinates_length_dict = (
            {}
        )  # Für schnellen Zugriff dict mit key = Länge und value = Array[(x, y)]

        # Loop over all free spaces and all neighbors and set distance of current
        # cell to 0 if neighbors have no distance (are obstacles) or to
        # nearest distance + 1 (cell is one more step away from obstacle than
        # neighbor)
        for x, y in free_space_coordinates:
            dist = float("inf")

            for j in range(-1, 2):
                for i in range(-1, 2):
                    if (i == 0 and j == 0) or x + j < 0 or y + i < 0:
                        continue

                    try:
                        val = map_2d[x + j, y + i]
                    except:
                        continue

                    if val != 0:
                        dist = 0
                        continue

                    index = self._get_index(x + j, y + i)

                    if coordinates_with_length[index] >= 0:
                        dist = min(coordinates_with_length[index] + 1, dist)

            if np.isinf(dist): dist = 0 #regularize

            coordinates_length_dict.setdefault(dist, []).append((x, y))
            coordinates_with_length[self._get_index(x, y)] = dist

        min_key, max_key = min(coordinates_length_dict.keys()), max(
            coordinates_length_dict.keys()
        )

        # Loop again over all cells from lowest to highest distance and update
        # all direct neighbors -> Set all distance to max current_dist + 1
        for key in range(min_key, max_key + 1):
            if not coordinates_length_dict.get(key):
                continue

            for x, y in coordinates_length_dict[key]:
                for j in range(-1, 2):
                    for i in range(-1, 2):
                        if (i == 0 and j == 0) or x + j < 0 or y + i < 0:
                            continue

                        try:
                            val = map_2d[x + j, y + i]
                        except:
                            ## Out of bounds
                            continue

                        index = self._get_index(x + j, y + i)

                        if coordinates_with_length[index] > key + 1:
                            coordinates_with_length[index] = key + 1
                            coordinates_length_dict.setdefault(key + 1, []).append(
                                (x + j, y + i)
                            )

        # print(list(coordinates_with_length))

        return coordinates_with_length
        # return np.reshape(coordinates_with_length, (height_in_cell, width_in_cell))

    def _get_index(self, x, y):
        return x * self.map.info.width + y


class DynamicMapDistanceServer(MapDistanceServer):
    def __init__(self):
        self._first_map = True
        # Publish a message to the MapGenerator that the new distance map is ready
        self.new_dist_map_pub = rospy.Publisher(
            "/signal_new_distance_map", String, queue_size=1
        )
        super().__init__()
        # Subscribe to the map topic to know when a new map is generated
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self._map_callback)

    def produce_distance_map(self):
        """Generates and saves or loads the distance map."""
        if self._first_map:
            # only update the map if it is the first map
            # as static server only contains the first map
            self.map = self.map_service().map

        if not os.path.exists(self._distance_map_path) or not self._first_map:
            # If the distance map does not exist or a new map is provided by map generator
            self.new_map_data = list(self._get_map_with_distances())
            self.new_dist_map_pub.publish(String(""))
            self.save_distance_map(
                self._distance_map_path, self.new_map_data, self.map.info
            )
        else:
            distance_map = self.get_distance_map(self._distance_map_path, self.map.info)
            self.new_map_data = distance_map

    def _map_callback(self, msg: OccupancyGrid):
        """Callback for when a new map is generated and published.

        Args:
            msg (OccupancyGrid): /map topic message.
        """
        if not self._first_map:
            # only update other than first map
            # override latest map data
            self.update_map_data(msg.data)
        self.produce_distance_map()
        self._first_map = False

    def update_map_data(self, new_map_data: list):
        """Updates the map data with the new distance map."""
        map_properties = rospy.get_param(MAP_GENERATOR_NS("map_properties"))
        width, height = map_properties["width"], map_properties["height"]

        required_map_size = len(self.map.data)
        if width * height != required_map_size:
            raise ValueError(
                "Map data and map properties do not match. "
                "New map must have the same size as the map properties. \n"
                f"Width x Height != {required_map_size} \n"
                f"{width} x {height} != {required_map_size}"
            )

        if not nearlyequal(map_properties["resolution"], self.map.info.resolution, 5):
            raise ValueError(
                "Map data and map properties do not match. "
                "New map must have the same resolution as the origin map. \n"
                f"Resolution != {self.map.info.resolution} \n"
                f"{map_properties['resolution']} != {self.map.info.resolution}"
            )

        # update the map info
        if width != self.map.info.width or height != self.map.info.height:
            self.map.info.width, self.map.info.height = width, height

        self.map.data = new_map_data


if __name__ == "__main__":
    rospy.init_node("map_distance_server")

    distance_server = (
        MapDistanceServer()
        if rospy.get_param("map_file") != "dynamic_map"
        else DynamicMapDistanceServer()
    )

    while not rospy.is_shutdown():
        rospy.spin()

    # map = map_service().map

    # width_in_cell, height_in_cell = map.info.width, map.info.height

    # map_2d = np.reshape(map.data, (height_in_cell, width_in_cell))

    # print_map(map_2d)

    # free_space_indices = np.where(map_2d == 0)

    # free_space_coordinates = np.array(free_space_indices).transpose()

    # # For every cell we calculate the distance to the nearest obstacle
    # # Therefore calculate the distance to every blocked cell and take the lowest
    # # distance. This is roughly n^2, but this is only done once so it is OK

    # coordinates_with_length = np.full(len(map.data), -1) ## Hier wird die Länge gespeichert
    # coordinates_length_dict = {} # Für schnellen Zugriff dict mit key = Länge und value = Array[(x, y)]

    # def get_index(x, y):
    #     return x * map.info.width + y

    # for x, y in free_space_coordinates:
    #     dist = float("inf")

    #     for j in range(-1, 2):
    #         for i in range(-1, 2):

    #             if (i == 0 and j == 0) or x + j < 0 or y + i < 0:
    #                 continue

    #             try:
    #                 val = map_2d[x + j, y + i]
    #             except:
    #                 continue

    #             if val != 0:
    #                 dist = 0
    #                 continue

    #             index = get_index(x + j, y + i)

    #             if coordinates_with_length[index] >= 0:
    #                 dist = min(coordinates_with_length[index] + 1, dist)

    #     coordinates_length_dict.setdefault(dist, []).append((x, y))
    #     coordinates_with_length[get_index(x, y)] = dist

    # print("DISTANCES")

    # min_key, max_key = min(coordinates_length_dict.keys()), max(coordinates_length_dict.keys())

    # for key in range(min_key, max_key + 1):

    #     if not coordinates_length_dict.get(key):
    #         continue

    #     for x, y in coordinates_length_dict[key]:

    #         for j in range(-1, 2):
    #             for i in range(-1, 2):
    #                 if (i == 0 and j == 0) or x + j < 0 or y + i < 0:
    #                     continue

    #                 try:
    #                     val = map_2d[x + j, y + i]
    #                 except:
    #                     ## Out of bounds
    #                     continue

    #                 index = get_index(x + j, y + i)

    #                 if coordinates_with_length[index] > key + 1:
    #                     coordinates_with_length[index] = key + 1
    #                     coordinates_length_dict.setdefault(key + 1, []).append((x + j, y + i))

    # print("finished", map.info, min(coordinates_with_length), max(coordinates_with_length))
