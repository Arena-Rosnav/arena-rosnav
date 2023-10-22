from typing import List, Optional, Tuple
import numpy as np
import random
import math


from map_distance_server.srv import GetDistanceMapResponse
from geometry_msgs.msg import Point
from task_generator.shared import Waypoint


class MapManager:
    """
    The map manager manages the static map
    and is used to get new goal, robot and
    obstacle positions.
    """

    _map: GetDistanceMapResponse
    _map_with_distances: np.ndarray
    _origin: Point
    _forbidden_zones: List[Waypoint]

    def __init__(self, map: GetDistanceMapResponse):
        self.update_map(map)
        self.init_forbidden_zones()

    def update_map(self, map: GetDistanceMapResponse):
        self._map = map
        self._map_with_distances = np.reshape(
            self._map.data,
            (self._map.info.height, self._map.info.width)
        )
        self._origin = map.info.origin.position

    def init_forbidden_zones(self, init: Optional[List[Waypoint]] = None):
        if init is None:
            init = list()

        self._forbidden_zones = init

    def forbid(self, forbidden_zones: List[Waypoint]):
        self._forbidden_zones += forbidden_zones

    def get_random_pos_on_map(self, safe_dist: float, forbid: bool = True, forbidden_zones: Optional[List[Waypoint]] = None) -> Waypoint:
        """
        This function is used by the robot manager and
        obstacles manager to get new positions for both
        robot and obstalces.
        The function will choose a position at random
        and then validate the position. If the position
        is not valid a new position is chosen. When
        no valid position is found after 100 retries
        an error is thrown.
        Args:
            safe_dist: minimal distance to the next
                obstacles for calculated positons
            forbid: add returned waypoint to forbidden zones
            forbidden_zones: Array of (x, y, radius),
                describing circles on the map. New
                position should not lie on forbidden
                zones e.g. the circles.
                x, y and radius are in meters
        Returns:
            A tuple with three elements: x, y, theta
        """
        # safe_dist is in meters so at first calc safe dist to distance on
        # map -> resolution of map is m / cell -> safe_dist in cells is
        # safe_dist / resolution
        safe_dist_in_cells = math.ceil(
            safe_dist / self._map.info.resolution) + 1

        forbidden_zones_in_cells: List[Waypoint] = [
            (
                math.ceil(point[0] / self._map.info.resolution),
                math.ceil(point[1] / self._map.info.resolution),
                math.ceil(point[2] / self._map.info.resolution)
            )
            for point in self._forbidden_zones + (forbidden_zones if forbidden_zones is not None else [])
        ]

        # Now get index of all cells were dist is > safe_dist_in_cells
        possible_cells: List[Tuple[np.intp, np.intp]] = np.array(
            np.where(self._map_with_distances > safe_dist_in_cells)).transpose().tolist()

        #return (random.randint(1,6), random.randint(1, 9), 0)
        assert len(possible_cells) > 0, "No cells available"

        # The position should not lie in the forbidden zones and keep the safe
        # dist to these zones as well. We could remove all cells here but since
        # we only need one position and the amount of cells can get very high
        # we just pick positions at random and check if the distance to all
        # forbidden zones is high enough

        while len(possible_cells) > 0:

            # Select a random cell
            x, y = possible_cells.pop(random.randrange(len(possible_cells)))

            # Check if valid
            if self._is_pos_valid(float(x), float(y), safe_dist_in_cells, forbidden_zones_in_cells):
                break

        else:
            raise Exception("can't find any non-occupied spaces")

        theta = random.uniform(-math.pi, math.pi)

        point: Waypoint = (
            float(np.round(y * self._map.info.resolution + self._origin.y, 3)),
            float(np.round(x * self._map.info.resolution + self._origin.x, 3)),
            theta
        )

        if forbid:
            self._forbidden_zones.append(point)

        return point

    def _is_pos_valid(self, x: float, y: float, safe_dist: float, forbidden_zones: List[Waypoint]):
        """
        @safe_dist: minimal distance to the next obstacles for calculated positions
        """
        for p in forbidden_zones:
            f_x, f_y, radius = p

            # euklidian distance to the forbidden zone
            dist = math.floor(np.linalg.norm(
                np.array([x, y]) - np.array([f_x, f_y]))) - radius

            if dist <= safe_dist:
                return False

        return True
