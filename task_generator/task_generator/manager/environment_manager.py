import itertools
import typing
from typing import Any, Callable, Collection, Iterator

import attrs
from task_generator import NodeInterface
from task_generator.simulators.human import BaseHumanSimulator
from task_generator.simulators.human.utils import ObstacleLayer
from task_generator.manager.world_manager.utils import World
from task_generator.shared import (DynamicObstacle, Entity, Obstacle,
                                   Orientation, Pose, Position, Robot, Wall)
from task_generator.simulators.sim import BaseSim

EntityPropsT = typing.TypeVar('EntityPropsT', bound=Entity)


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

    def _realize_position(self, position: Position) -> Position:
        position.x += self._config.x
        position.y += self._config.y
        return position

    def _realize_orientation(self, orientation: Orientation) -> Orientation:
        return orientation

    def _realize_pose(self, pose: Pose) -> Pose:
        pose.position = self._realize_position(pose.position)
        return pose

    @typing.overload
    def realize(self, target: EntityPropsT) -> EntityPropsT: ...

    @typing.overload
    def realize(self, target: Position) -> Position: ...

    @typing.overload
    def realize(self, target: Pose) -> Pose: ...

    def _realize_entity(self, entity: EntityPropsT) -> EntityPropsT:
        entity.pose = self._realize_pose(entity.pose)
        entity.name = self._prefix(entity.name)
        return entity

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

        if isinstance(target, Pose):
            return self._realize_pose(target)

        if isinstance(target, Entity):
            return self._realize_entity(target)

        if isinstance(target, Wall):
            return self._realize_wall(target)

        raise TypeError(f'realization not implemented for type {type(target)}')


class EnvironmentManager(NodeInterface, _Realizer):

    _namespace: str
    _entity_manager: BaseHumanSimulator
    _simulator: BaseSim

    id_generator: Iterator[int]

    def __init__(
        self,
        namespace,
        simulator: BaseSim,
        entity_manager: BaseHumanSimulator,
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

    def move_robot(self, name: str, pose: Pose):
        """
        Moves given robot
        """
        self._entity_manager.move_robot(
            name=name,
            pose=self._realize_pose(pose),
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
