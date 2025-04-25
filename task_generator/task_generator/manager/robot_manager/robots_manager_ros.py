import abc
import os
import re
import typing

import arena_simulation_setup
import attrs
import rclpy
import rclpy.parameter
import yaml
from ament_index_python import get_package_share_directory

from task_generator import NodeInterface
from task_generator.manager.entity_manager import EntityManager
from task_generator.shared import Namespace, PositionOrientation, Robot
from task_generator.utils import ModelLoader
from task_generator.utils.ros_params import ROSParam

from .robot_manager import RobotManager


def _initialpose_generator(x: float, y: float, d: float):
    while True:
        yield PositionOrientation(x=x, y=y, orientation=0)
        y += d


@attrs.frozen
class _RobotDiff:
    to_remove: list[str] = attrs.field(factory=list)
    to_add: dict[str, Robot] = attrs.field(factory=dict)
    to_update: dict[str, Robot] = attrs.field(factory=dict)


_robot_loader = ModelLoader(arena_simulation_setup.Robot.base_dir())


class RobotsManager(abc.ABC):

    _entity_manager: EntityManager
    _robot_managers: dict[str, RobotManager]

    @property
    def robot_managers(self) -> dict[str, RobotManager]:
        return self._robot_managers

    @abc.abstractmethod
    def set_up(self):
        ...

    def __init__(self, entity_manager: EntityManager) -> None:
        self._entity_manager = entity_manager
        self._robot_managers = {}


class RobotsManagerROS(NodeInterface, RobotsManager):
    """
    ROS interface for dynamically loading multiple robots.
    """

    _initialpose: typing.Generator
    _robot_configurations: ROSParam[_RobotDiff]
    _diff: _RobotDiff

    @classmethod
    def _parse_setup_file(
        cls,
        setup_file: str
    ) -> typing.Iterable[dict]:
        with open(
            os.path.join(
                get_package_share_directory('arena_bringup'),
                'configs',
                'robot_setup',
                setup_file
            )
        ) as f:
            configuration = yaml.load(f, yaml.SafeLoader)

        result: list[dict] = []

        for entry_ in configuration:
            entry: dict = dict(entry_)
            amount = int(entry.pop('amount', 1))
            for _ in range(amount):
                result.append(entry)

        return result

    def _parse_robot_configurations(
        self,
        v: typing.Any
    ) -> _RobotDiff:

        robot_arg: list[str] = str(v).split(',')

        parsed_explicit: dict[str, Robot] = {}
        parsed_anonymous: dict[str, list[Robot]] = {}

        def add(base: dict):
            name = base.get('name')
            config = Robot.parse(base, _robot_loader.bind(base['model']))

            if name is None:  # anon
                parsed_anonymous.setdefault(
                    config.model.name, []
                ).append(config)

            else:  # explicit name
                if name in parsed_explicit:
                    raise RuntimeError(
                        f'naming conflict for robots with name {name}')

                parsed_explicit[name] = config

        for arg in robot_arg:
            if arg.endswith('.yaml'):
                # load robot_setup_file from arena_bringup/configs/robot_setup
                for addition in self._parse_setup_file(arg):
                    add(addition)
            elif (match := re.match(r'(.*)\[(\d+)\]', arg)):
                # multi-instantiations via model[count]
                for _ in range(int(match.group(2))):
                    add(dict(model=match.group(1)))
            else:
                # just model
                add(dict(model=arg))

        existing = {
            k: v.robot
            for k, v
            in self._robot_managers.items()
        }

        existing_keys = set(existing.keys())
        matchable_keys = set(existing_keys)

        to_add: dict[str, Robot] = {}
        to_update: dict[str, Robot] = {}

        # explicit naming first
        for prefix, config in parsed_explicit.items():
            match = next(
                (key for key in matchable_keys if existing[key] == config),
                None
            )

            if match is None:  # no matches
                to_add[prefix] = config
            else:  # exact match
                matchable_keys.remove(match)

        # anonymous naming
        for prefix, configs in parsed_anonymous.items():
            unassigned: list[Robot] = []

            for config in configs:
                match = next(
                    (
                        key
                        for key
                        in matchable_keys
                        if existing[key].compatible(config)
                    ),
                    None
                )

                if match is None:  # no similar robot found
                    unassigned.append(config)
                else:  # similar robot found, update
                    to_update[match] = config
                    matchable_keys.remove(match)

            i: int = 0
            for anon in unassigned:
                suffixed_key = prefix

                if len(configs) > 1:
                    suffixed_key = f'{prefix}_{i}'

                while suffixed_key in existing_keys:
                    i += 1
                    suffixed_key = f'{prefix}_{i}'

                to_add[suffixed_key] = anon
                existing_keys.add(suffixed_key)

        self._diff = _RobotDiff(
            to_remove=list(matchable_keys),
            to_add=to_add,
            to_update=to_update,
        )
        return self._diff

    def set_up(self):

        for robot_name in self._diff.to_remove:
            self._robot_managers.pop(robot_name).destroy()
        self._diff.to_remove.clear()

        for robot_name, config in self._diff.to_update.items():
            self._robot_managers[robot_name].update()
            # TODO
        self._diff.to_update.clear()

        for robot_name, config in self._diff.to_add.items():
            manager = RobotManager(
                namespace=Namespace(
                    self.node.get_namespace())(
                        self.node.get_name(),
                ),
                environment_manager=self._entity_manager,
                robot=attrs.evolve(
                    config,
                    name=robot_name,
                    position=next(self._initialpose)
                ),
            )
            manager.set_up_robot()
            self._robot_managers[robot_name] = manager
        self._diff.to_add.clear()

        self.node.rosparam[list[str]].set('robot_names', [robot.name for robot in self._robot_managers.values()])

    def __init__(self, entity_manager: EntityManager) -> None:
        NodeInterface.__init__(self)
        RobotsManager.__init__(self, entity_manager=entity_manager)
        self._initialpose = _initialpose_generator(-10, -10, -5)

        self._robot_configurations = self.node.ROSParam[_RobotDiff](
            'robot',
            type_=rclpy.parameter.Parameter.Type.STRING,
            parse=self._parse_robot_configurations,
        )
