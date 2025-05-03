import itertools
import typing
from typing import Any, Callable, Collection, Iterator

import attrs

from task_generator import NodeInterface
from task_generator.manager.entity_manager import EntityManager
from task_generator.manager.entity_manager.utils import ObstacleLayer
from task_generator.manager.world_manager.utils import World
from task_generator.shared import (DynamicObstacle, Entity, Obstacle, Position,
                                   Robot, Wall)
from task_generator.simulators import BaseSimulator

EntityPropsT = typing.TypeVar('EntityPropsT', bound=Entity)
PositionT = typing.TypeVar('PositionT', bound=Position)


class _Realizer:
    @attrs.frozen()
    class _Configuration:
        x: float = 0.0
        y: float = 0.0
        prefix: str = ''

    _config: "_Configuration"

    @typing.overload
    def realize(self, target: str) -> str: ...

    def _prefix(self, s: str) -> str:
        return self._config.prefix + s

    @typing.overload
    def realize(self, target: PositionT) -> PositionT: ...

    def _realize_position(self, position: PositionT) -> PositionT:
        return attrs.evolve(
            position,
            x=position.x + self._config.x,
            y=position.y + self._config.y,
        )

    @typing.overload
    def realize(self, target: EntityPropsT) -> EntityPropsT: ...

    def _realize_entity(self, entity: EntityPropsT) -> EntityPropsT:
        return attrs.evolve(
            entity,
            position=self._realize_position(entity.position),
            name=self._prefix(entity.name),
        )

    @typing.overload
    def realize(self, target: Wall) -> Wall: ...

    def _realize_wall(self, wall: Wall) -> Wall:
        return attrs.evolve(
            wall,
            Start=self._realize_position(wall.Start),
            End=self._realize_position(wall.End),
        )

    def realize(
        self,
        target
    ):
        if isinstance(target, str):
            return self._prefix(target)

        if isinstance(target, Position):
            return self._realize_position(target)

        if isinstance(target, Entity):
            return self._realize_entity(target)

        if isinstance(target, Wall):
            return self._realize_wall(target)

        raise TypeError(f'realization not implemented for type {type(target)}')


class EnvironmentManager(NodeInterface, _Realizer):

    _namespace: str
    _entity_manager: EntityManager
    _simulator: BaseSimulator

    id_generator: Iterator[int]

    def __init__(
        self,
        namespace,
        simulator: BaseSimulator,
        entity_manager: EntityManager,
    ):
        NodeInterface.__init__(self)

        self._namespace = namespace
        self._simulator = simulator
        self._entity_manager = entity_manager

        ref_x, ref_y = self.node.rosparam[tuple[float, float]].get('reference', [0.0, 0.0])
        prefix = self.node.rosparam[str].get('prefix', '')
        self._config = self._Configuration(
            x=ref_x,
            y=ref_y,
            prefix=prefix,
        )

        self.id_generator = itertools.count(434)

    def spawn_world_obstacles(self, world: World):
        """
        Loads given obstacles into the simulator,
        the map file is retrieved from launch parameter "world"
        """

        walls = world.entities.walls

        if walls:
            self._entity_manager.spawn_walls(list(map(self._realize_wall, walls)))
        self._entity_manager.spawn_obstacles(list(map(self._realize_entity, world.entities.obstacles)))

    def spawn_dynamic_obstacles(self, setups: Collection[DynamicObstacle]):
        """
        Loads given dynamic obstacles into the simulator.
        """

        self._entity_manager.spawn_dynamic_obstacles(obstacles=list(map(self._realize_entity, setups)))

    def spawn_obstacles(self, setups: Collection[Obstacle]):
        """
        Loads given obstacles into the simulator.
        """

        self._entity_manager.spawn_obstacles(obstacles=list(map(self._realize_entity, setups)))

    def spawn_robot(self, robot: Robot) -> Robot:
        """
        Loads given robot into the simulator
        """
        robot = self._realize_entity(robot)
        self._entity_manager.spawn_robot(robot)
        return robot

    def move_robot(self, name: str, position: Position):
        """
        Moves given robot
        """
        self._entity_manager.move_robot(
            name=name,
            position=self._realize_position(position),
        )

    def respawn(self, callback: Callable[[], Any]):
        """
        Unuse obstacles, (re-)use them in callback, finally remove unused obstacles
        @callback: Function to call between unuse and remove
        """
        self._entity_manager.unuse_obstacles()
        callback()
        self._entity_manager.remove_obstacles(purge=ObstacleLayer.UNUSED)

    def reset(self, purge: ObstacleLayer = ObstacleLayer.INUSE):
        """
        Unuse and remove all obstacles
        """
        self._entity_manager.remove_obstacles(purge=purge)
