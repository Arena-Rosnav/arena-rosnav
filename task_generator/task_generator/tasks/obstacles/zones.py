import itertools
import pathlib
import numpy as np
import regex
from typing import Callable, Dict, Iterator, List
import typing

import yaml
import rospkg
import rospy
from task_generator.constants import Config, Constants
from task_generator.shared import (
    DynamicObstacle,
    PositionOrientation,
    PositionRadius,
    Position,
    rosparam_get
)
from task_generator.tasks.obstacles import Obstacles, TM_Obstacles
from task_generator.tasks.obstacles.utils import ITF_Obstacle
from task_generator.tasks.task_factory import TaskFactory

import dynamic_reconfigure.client
import dataclasses
import shapely
import typing_extensions

import pydantic
import pydantic.functional_validators

T = typing.TypeVar("T")
U = typing.TypeVar("U")

# good enough for me
def try_parse(_as: typing.Type[T], default: U, chain: typing.Iterable[typing.Callable[..., T]]) -> typing.Callable[..., typing.Union[T,U]]:
    def parse(obj: typing.Any) -> typing.Union[T, U]:
        for c in chain:
            try:
                res = c(obj)
                if not isinstance(res, _as): raise TypeError
                return res
            except Exception as e:
                pass
        else:
            return default                
    return parse


# const | (min,max) -> (const, const) | (min, max)
_ConstantOrRange = typing_extensions.Annotated[
    typing.Tuple[T, T],
    pydantic.functional_validators.BeforeValidator(
        try_parse(
            typing.Tuple,
            None,
            (
                tuple,
                lambda v: (v,v)
            )
        )
    )
]

# single | [multiple, ...] -> [single] | [multiple, ...]
_ConstantOrList = typing_extensions.Annotated[
    typing.List[T],
    pydantic.functional_validators.BeforeValidator(
        lambda v: v if isinstance(v, list) else [v]
    )
]



# polygon | [polygon, ...] -> shapely.MultiPolygon
_Multipolygon = typing_extensions.Annotated[
    shapely.MultiPolygon,
    pydantic.functional_validators.BeforeValidator(
        try_parse(
            _as=shapely.MultiPolygon,
            default=None,
            chain=(
                lambda v: shapely.MultiPolygon(v),
                lambda v: shapely.MultiPolygon([shapely.Polygon(p) for p in v]),
            )
        )
    )
]


def _rekey(d: typing.Dict[str, typing.Any]) -> typing.Dict[str, typing.Any]:
    """
    Convert dict keys to valid python identifiers
    """
    # https://stackoverflow.com/a/66493506

    ID_START_REGEX = (
        r'\p{Lu}\p{Ll}\p{Lt}\p{Lm}\p{Lo}\p{Nl}' r'_\u1885-\u1886\u2118\u212E\u309B-\u309C')
    ID_CONTINUE_REGEX = ID_START_REGEX + \
        (r'\p{Mn}\p{Mc}\p{Nd}\p{Pc}' r'\u00B7\u0387\u1369-\u1371\u19DA')

    def regularize(k: str) -> str:
        k = regex.sub(fr"^[^{ID_CONTINUE_REGEX}]*", "", k, flags=regex.UNICODE)
        k = regex.sub(fr"[^{ID_CONTINUE_REGEX}]+", "_", k, flags=regex.UNICODE)
        return k

    return {regularize(k): v for k, v in d.items()}


@dataclasses.dataclass
class _Config:

    @pydantic.dataclasses.dataclass
    class Entity:
        name: str
        type: str
        model: _ConstantOrList[str]
        waypoints_in: typing.Union[typing.Literal["*"], typing.Set[str]]
        number_of_waypoints: _ConstantOrRange[pydantic.types.PositiveInt]
        amount: _ConstantOrRange[pydantic.types.NonNegativeInt]

    @pydantic.dataclasses.dataclass(config={"arbitrary_types_allowed":True})
    class Zone:
        label: str
        category: _ConstantOrList[str]
        polygon: _Multipolygon

    SCENARIO: typing.List[Entity] = dataclasses.field(default_factory=list)
    ZONES: typing.List[Zone] = dataclasses.field(default_factory=list)

    SCENARIO_FILE: pydantic.FilePath = pathlib.Path()
    ZONES_FILE: pydantic.FilePath = pathlib.Path()


@TaskFactory.register_obstacles(Constants.TaskMode.TM_Obstacles.ZONES)
class TM_Zones(TM_Obstacles):
    """
    Zones task generator for obstacles.

    This class generates random obstacles with zones for a task scenario.

    Attributes:
        _config (Config): Configuration object for obstacle generation.

    Methods:
        prefix(*args): Prefixes the given arguments with "zones".
        __init__(**kwargs): Initializes the TM_Random object.
        reconfigure(config): Reconfigures the obstacle generation based on the given configuration.
        reset(**kwargs): Resets the obstacle generation with the specified parameters.

    """

    _config: _Config

    @classmethod
    def prefix(cls, *args):
        return super().prefix("zones", *args)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self._config = _Config()

        dynamic_reconfigure.client.Client(
            name=self.NODE_CONFIGURATION, config_callback=self.reconfigure
        )

    def reconfigure(self, config):
        """
        Reconfigures the obstacle parameters based on the provided configuration.

        Args:
            config: The configuration object containing the obstacle parameters.

        Returns:
            None
        """

        world_dir = pathlib.Path(rospkg.RosPack().get_path("arena_simulation_setup")) / "worlds" / rosparam_get(str, "map_file")

        # updated zones file if changed
        zones_file = world_dir / "map" / "zones" / config.get("ZONES_file", "../zones.yaml")
        zones_file = pathlib.Path.resolve(zones_file, strict=False)
        if self._config.ZONES_FILE != zones_file:
            try:
                with open(zones_file) as f:
                    zones = [_Config.Zone(**_rekey(z)) for z in yaml.safe_load(f)]
            except (pydantic.ValidationError, FileNotFoundError, yaml.YAMLError) as e:
                rospy.logerr(e)
                rospy.logwarn(
                    f"failed to change zones (remains {self._config.ZONES_FILE})")
            else:
                self._config.ZONES_FILE = zones_file
                self._config.ZONES = zones
                rospy.loginfo(
                    f"zones scenario changed to {self._config.ZONES_FILE}")

        # update scenario file if changed
        scenario_file = world_dir / "zones" / config.get("ZONES_scenario")
        if self._config.SCENARIO_FILE != scenario_file:
            try:
                with open(scenario_file) as f:
                    scenario = [_Config.Entity(**_rekey(e)) for e in yaml.safe_load(f)]
            except (pydantic.ValidationError, FileNotFoundError, yaml.YAMLError) as e:
                rospy.logerr(e)
                rospy.logwarn(
                    f"failed to change scenario (remains {self._config.SCENARIO_FILE})")
            else:
                self._config.SCENARIO_FILE = scenario_file
                self._config.SCENARIO = scenario
                rospy.loginfo(
                    f"zones scenario changed to {self._config.SCENARIO_FILE}")

    def reset(self, **kwargs) -> Obstacles:
        """
        Resets the obstacle generation with the specified parameters.

        Args:
            **kwargs: Additional keyword arguments for customizing the obstacle generation.

        Returns:
            Tuple[[], List[DynamicObstacle]]: A tuple containing the generated obstacles (none) and dynamic obstacles.

        """

        def indexer() -> Callable[..., int]:
            indices: Dict[str, Iterator[int]] = dict()

            def index(model: str):
                if model not in indices:
                    indices[model] = itertools.count(1)
                return next(indices[model])

            return index

        # TODO move this to reconfig

        # reverse index category -> [zone] with choice function
        class _Categories(typing.Dict[str, typing.List[int]]):

            _all_zones: typing.Set[int]

            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self._all_zones = set()

            def add(self, category: str, zone: int):
                self._all_zones.add(zone)
                self.setdefault(category, []).append(zone)

            def random(self, category: str) -> int:
                target = self.get(category, [])
                if not len(target): return self.any()
                return target[rng.integers(0, len(target))]
            
            def any(self) -> int:
                all = list(self._all_zones)
                return all[rng.integers(0, len(all))]

        categories = _Categories()
        for i, zone in enumerate(self._config.ZONES):
            for category in zone.category:
                categories.add(category, i)

        # END TODO

        rng = Config.General.RNG

        # how many waypoints in each zone
        claims: Dict[int, int] = {i: 0 for i, zone in enumerate(self._config.ZONES)}

        # [RNG] how many entities per kind
        n_entities: List[int] = list()

        # [RNG] how many waypoints per entity
        n_waypoints: List[int] = list()

        # RNG number of entites & waypoints, lay claims on zones
        for entity in self._config.SCENARIO:
            n_entities.append(n_entity := rng.integers(
                *entity.amount, endpoint=True))

            for n_waypoint in rng.integers(*entity.number_of_waypoints, endpoint=True, size=n_entity):
                n_waypoints.append(n_waypoint)
                n_waypoint += 1 # for start position

                targets = list(categories.keys() if entity.waypoints_in == "*" else entity.waypoints_in)
                for category in rng.choice(targets, size=n_waypoint):
                    claims[categories.random(category)] += 1

        # fit waypoints
        all_positions: typing.List[Position] = list()

        for zone_index, n in claims.items():
            zone_positions: typing.List[Position] = list()

            zone = self._config.ZONES[zone_index]
            poly = shapely.MultiPolygon(zone.polygon)

            min_x, min_y, max_x, max_y = shapely.bounds(poly)
            bounds = self._PROPS.world_manager.Bounds(
                min_x = min_y,
                min_y = min_x,
                max_x = max_y,
                max_y = max_x
            )

            for _ in range(1):
                if (to_produce := n - len(zone_positions)) <= 0:
                    break

                zone_positions += [
                    point
                    for point
                    in self._PROPS.world_manager.positions_on_map(
                        n=to_produce,
                        safe_dist=0,
                        forbid=False,
                        bounds=bounds
                    )
                    if print(shapely.contains_xy(poly, point.x, point.y), poly, point) or shapely.contains_xy(poly, point.x, point.y)
                ]
            else:
                # lost cause
                to_produce = n - len(zone_positions)
                # rospy.logwarn(f"{to_produce} waypoints could not be placed")
                zone_positions += self._PROPS.world_manager.garbage_positions(to_produce)

            all_positions += zone_positions[:n]

        # assign obstacles
        i_waypoints = iter(n_waypoints)
        i_positions = iter(all_positions)

        index = indexer()
        dynamic_obstacles: List[DynamicObstacle] = []

        for entity, n_entity in zip(self._config.SCENARIO, n_entities):

            dynamic_obstacles += [
                ITF_Obstacle.create_dynamic_obstacle(
                    self._PROPS,
                    name=f"{entity.name}_{index(entity.name)}",
                    model=self._PROPS.dynamic_model_loader.bind(model),
                    waypoints=[
                        PositionRadius(*p, radius=1)
                        for p in
                        itertools.islice(i_positions, next(i_waypoints))
                    ],
                    position=PositionOrientation(*next(i_positions), 2*np.pi*rng.random()),
                )
                for model in Config.General.RNG.choice(
                    a=entity.model,
                    size=n_entity,
                )
            ]

        return [], dynamic_obstacles
