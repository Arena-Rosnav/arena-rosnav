from task_generator.shared import Position, PositionRadius
import itertools
from math import floor
from typing import Collection, List, Optional, Tuple
import numpy as np
import scipy.signal
from task_generator import NodeInterface

from .utils import World, WorldEntities, WorldMap, WorldObstacleConfiguration, WorldObstacleConfigurations, WorldOccupancy, WorldWalls, WorldZones, configurations_to_obstacles, occupancy_to_walls


class WorldManager(NodeInterface):
    """
    The map manager manages the static map
    and is used to get new goal, robot and
    obstacle positions.
    """

    _world: World
    _classic_forbidden_zones: List[PositionRadius]

    def __init__(
        self
    ):
        NodeInterface.__init__(self)
        self._detected_walls = None
        self._classic_forbidden_zones = []

    @property
    def world(self) -> World:
        return self._world

    @property
    def _shape(self) -> Tuple[int, int]:
        return self._world.map.shape[0], self._world.map.shape[1]

    @property
    def origin(self) -> Position:
        return self._world.map.origin

    @property
    def resolution(self) -> float:
        return self._world.map.resolution

    @property
    def walls(self) -> WorldWalls:
        return self._world.entities.walls

    _detected_walls: Optional[WorldWalls]

    @property
    def detected_walls(self) -> WorldWalls:
        if self._detected_walls is None:
            self._detected_walls = occupancy_to_walls(
                occupancy_grid=self._world.map.occupancy._walls.grid,
                transform=self._world.map.tf_grid2pos
            )
        return self._detected_walls

    @property
    def zones(self) -> WorldZones:
        return self.world.zones

    def update_world(
        self,
        world_map: WorldMap,
        obstacles: Optional[WorldObstacleConfigurations] = None,
        walls: Optional[WorldWalls] = None,
        zones: Optional[WorldZones] = None,
    ):
        self._detected_walls = None

        if obstacles is None:
            obstacles = []

        if walls is None:
            walls = []

        if zones is None:
            zones = []

        parsed_obstacles = configurations_to_obstacles(
            configurations=obstacles
        )

        entities = WorldEntities(
            obstacles=parsed_obstacles,
            walls=walls
        )

        self._world = World(
            entities=entities,
            map=world_map,
            zones=zones,
        )

        for obstacle in self.world.entities.obstacles:
            self.world.map.occupancy.obstacle_occupy(
                *self.world.map.tf_posr2rect(
                    PositionRadius(
                        x=obstacle.position.x,
                        y=obstacle.position.y,
                        radius=1,   # TODO actual radius
                    )
                )
            )

    def forbid(self, forbidden_zones: List[PositionRadius]):
        for zone in forbidden_zones:
            self.world.map.occupancy.forbidden_occupy(
                *self.world.map.tf_posr2rect(zone))

    def forbid_clear(self):
        self._world.map.occupancy.forbidden_clear()

    def _classic_get_random_pos_on_map(self, safe_dist: float, forbid: bool = True,
                                       forbidden_zones: Optional[List[PositionRadius]] = None) -> Position:
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

        import math

        def is_pos_valid(x: float, y: float, safe_dist: float,
                         forbidden_zones: List[PositionRadius]):
            """
            @safe_dist: minimal distance to the next obstacles for calculated positions
            """
            for p in forbidden_zones:
                # euklidian distance to the forbidden zone
                dist = math.floor(np.linalg.norm(
                    np.array([x, y]) - np.array([p.x, p.y]))) - p.radius

                if dist <= safe_dist:
                    return False

            return True

        safe_dist_in_cells = math.ceil(
            safe_dist / self.world.map.resolution) + 1

        forbidden_zones_in_cells: List[PositionRadius] = [
            PositionRadius(
                x=math.ceil(point.x / self.world.map.resolution),
                y=math.ceil(point.y / self.world.map.resolution),
                radius=math.ceil(point.radius / self.world.map.resolution),
            )
            for point in self._classic_forbidden_zones + (forbidden_zones if forbidden_zones is not None else [])
        ]

        # Now get index of all cells were dist is > safe_dist_in_cells
        possible_cells: List[Tuple[np.intp, np.intp]] = np.array(
            np.where(self.world.map.occupancy.grid > safe_dist_in_cells)).transpose().tolist()

        # return (random.randint(1,6), random.randint(1, 9), 0)
        assert len(possible_cells) > 0, "No cells available"

        # The position should not lie in the forbidden zones and keep the safe
        # dist to these zones as well. We could remove all cells here but since
        # we only need one position and the amount of cells can get very high
        # we just pick positions at random and check if the distance to all
        # forbidden zones is high enough

        while len(possible_cells) > 0:

            # Select a random cell
            x, y = possible_cells.pop(
                self.node.conf.General.RNG.integers(
                    len(possible_cells)))

            # Check if valid
            if is_pos_valid(float(x), float(y), safe_dist_in_cells,
                            forbidden_zones_in_cells):
                break

        else:
            raise RuntimeError("can't find any non-occupied spaces")

        point = PositionRadius(
            x=np.round(float(x) * self.world.map.resolution + self.world.map.origin.x, 3),
            y=np.round(float(y) * self.world.map.resolution + self.world.map.origin.y, 3),
            radius=safe_dist,
        )

        if forbid:
            self._classic_forbidden_zones.append(point)

        return Position(x=point.x, y=point.y)

    def get_positions_on_map(
        self,
        n: int, safe_dist: float,
        forbidden_zones: Optional[List[PositionRadius]] = None,
        forbid: bool = True
    ) -> List[Position]:
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
        """
        # safe_dist is in meters so at first calc safe dist to distance on
        # map -> resolution of map is m / cell -> safe_dist in cells is
        # safe_dist / resolution

        if forbidden_zones is None:
            forbidden_zones = []

        fork = self._world.map.occupancy.fork()

        points: List[Position] = []

        if n < 0:  # TODO profile when this is faster
            for _ in range(n):
                pos = self._classic_get_random_pos_on_map(
                    safe_dist=safe_dist, forbidden_zones=forbidden_zones)
                posr = PositionRadius(x=pos.x, y=pos.y, radius=safe_dist)
                fork.occupy(*self.world.map.tf_posr2rect(posr))
                forbidden_zones.append(posr)
                points.append(pos)

            return points

        else:
            max_depth = 10

            for zone in forbidden_zones:
                fork.occupy(
                    *self.world.map.tf_posr2rect(
                        PositionRadius(
                            x=zone.x,
                            y=zone.y,
                            radius=zone.radius / self.resolution,
                        )
                    )
                )

            min_dist: float = safe_dist / self.resolution
            available_positions = self._occupancy_to_available(
                occupancy=fork.grid, safe_dist=min_dist)

            def sample(target: int) -> Collection[Position]:

                all_banned: np.ndarray = np.zeros((target, 2))
                banned_index: int = 0

                result: List[Position] = list()
                depth: int = 0

                to_produce = target

                try:
                    while depth < max_depth:

                        if to_produce > len(available_positions):
                            raise RuntimeError()

                        candidates = available_positions[self.node.conf.General.RNG.value.choice(
                            len(available_positions), to_produce, replace=False), :]

                        for candidate in candidates:

                            banned = all_banned[:banned_index, :]

                            if np.any(np.linalg.norm(
                                    banned - candidate, axis=1) < min_dist):
                                continue

                            all_banned[banned_index] = candidate
                            banned_index += 1

                            fork.occupy(
                                (candidate - min_dist),
                                (candidate + min_dist)
                            )

                            result.append(self._world.map.tf_grid2pos(
                                (candidate[0], candidate[1])))

                        to_produce = target - len(result)
                        if to_produce <= 0:
                            break

                        depth += 1

                    else:
                        raise RuntimeError(
                            f"Failed to find free position after {depth} tries")

                except RuntimeError:
                    result += [
                        self._world.map.tf_grid2pos(
                            (
                                (-1 - floor(i / 5)) * int(self._shape[1] / 5),
                                int((i % 5) * self._shape[0] / 5)
                            )
                        ) for i in range(to_produce)]
                    self.node.get_logger().warn(
                        f"Couldn't find enough empty cells for {to_produce} requests")

                return result

            points = list(sample(n))

        if forbid:
            fork.commit()

        return points

    def get_position_on_map(
            self, safe_dist: float, forbidden_zones: Optional[List[PositionRadius]] = None, forbid: bool = True) -> Position:
        return self.get_positions_on_map(
            n=1, safe_dist=safe_dist, forbidden_zones=forbidden_zones, forbid=forbid
        )[0]

    id_gen = itertools.count()

    def _occupancy_to_available(
            self, occupancy: np.ndarray, safe_dist: float) -> np.ndarray:

        filt_size = int(2 * safe_dist + 1)
        filt = np.full((filt_size, filt_size), 1) / (filt_size ** 2)

        spread = scipy.signal.convolve2d(
            WorldOccupancy.not_full(occupancy).astype(
                np.uint8) * np.iinfo(np.uint8).max,
            filt,
            mode="full",
            boundary="fill",
            fillvalue=int(WorldOccupancy.FULL)
        )

        # import cv2

        # def visual(mat: np.ndarray) -> np.ndarray:
        #     return (mat / mat.max()).astype(np.uint8) * np.iinfo(np.uint8).max

        # cv2.imwrite("_debug1.png", visual(occupancy))
        # cv2.imwrite("_debug2.png", visual(WorldOccupancy.not_full(occupancy)))
        # cv2.imwrite("_debug3.png", visual(spread))
        # cv2.imwrite("_debug4.png", visual(WorldOccupancy.empty(spread)))

        return np.transpose(np.where(WorldOccupancy.empty(spread)))
