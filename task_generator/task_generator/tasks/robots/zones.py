import math
from typing import List, Dict, Tuple

import genpy
import rospy
import pathlib
import pydantic
import yaml
import dynamic_reconfigure.client

from task_generator.constants import Config, Constants
from task_generator.shared import PositionOrientation, PositionRadius
from task_generator.tasks.task_factory import TaskFactory
from task_generator.tasks.robots import TM_Robots
from task_generator.tasks.obstacles.zones import Configuration, getPositions, rekey

@TaskFactory.register_robots(Constants.TaskMode.TM_Robots.ZONES)
class TM_Zones(TM_Robots):
    """
    This class represents a task manager for exploring robots.
    It inherits from the TM_Random class.
    """

    _timeouts: Dict[int, genpy.Time]

    _roles: Dict[int, Configuration.Role]

    _config: Configuration

    _categories: Dict[str, List[int]]

    @classmethod
    def prefix(cls, *args):
        return super().prefix("zones", *args)
        

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self._config = Configuration()

        dynamic_reconfigure.client.Client(
            name=self.NODE_CONFIGURATION, config_callback=self.reconfigure
        )
    
        self._timeouts = dict()
        self._roles = dict()
        for i in range(len(self._PROPS.robot_managers)):
            self._reset_timeout(i)
            self._set_role(i)
        
        self._categories = dict()
        for i, zone in enumerate(self._config.ZONES):
            for category in zone.category:
                if category in self._categories:
                    self._categories[category].append(i)
                else:
                    self._categories[category] = [i]
        
        
    def reconfigure(self, config):
        """
        Reconfigures the obstacle parameters based on the provided configuration.

        Args:
            config: The configuration object containing the obstacle parameters.

        Returns:
            None
        """

        world_dir = pathlib.Path(Constants.MapGenerator.WORLD_FOLDER)

        # updated zones file if changed
        zones_file = world_dir / "map" / "zones" / config.get("ZONES_file", "../zones.yaml")
        zones_file = pathlib.Path.resolve(zones_file, strict=False)
        if self._config.ZONES_FILE != zones_file:
            try:
                with open(zones_file) as f:
                    zones = [Configuration.Zone(**rekey(z)) for z in yaml.safe_load(f)]
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
                    scenario : Dict[str, List] = yaml.safe_load(f)
                    roles = [Configuration.Role(**rekey(e)) for e in scenario.get("robots", [])]
            except (pydantic.ValidationError, FileNotFoundError, yaml.YAMLError) as e:
                rospy.logerr(e)
                rospy.logwarn(
                    f"failed to change scenario (remains {self._config.SCENARIO_FILE})")
            else:
                self._config.SCENARIO_FILE = scenario_file
                self._config.ROLES = roles
                rospy.loginfo(
                    f"zones scenario changed to {self._config.SCENARIO_FILE}")

    def reset(self, **kwargs):
        """
        Reset the task generator.

        Args:
            **kwargs: Additional keyword arguments.

        Returns:
            None
        """

        super().reset(**kwargs)

        ROBOT_POSITIONS: List[
            Tuple[PositionOrientation, PositionOrientation]
        ] = kwargs.get("ROBOT_POSITIONS", [])
        biggest_robot = max(robot.safe_distance for robot in self._PROPS.robot_managers)

        for i in range(len(self._PROPS.robot_managers)):
            self._set_role(i)

        self._categories = dict()
        for i, zone in enumerate(self._config.ZONES):
            for category in zone.category:
                if category in self._categories:
                    self._categories[category].append(i)
                else:
                    self._categories[category] = [i]

        for robot_start, robot_goal in ROBOT_POSITIONS:
            self._PROPS.world_manager.forbid(
                [
                    PositionRadius(robot_start.x, robot_start.y, biggest_robot),
                    PositionRadius(robot_goal.x, robot_goal.y, biggest_robot),
                ]
            )

        if len(ROBOT_POSITIONS) < len(self._PROPS.robot_managers):

            for i, _ in enumerate(self._PROPS.robot_managers[len(ROBOT_POSITIONS):]):
                start = self._get_position(i)
                goal = self._get_position(i)
                ROBOT_POSITIONS.append((start, goal))
        
        print(ROBOT_POSITIONS)

        for robot, pos in zip(self._PROPS.robot_managers, ROBOT_POSITIONS):
            robot.reset(start_pos=pos[0], goal_pos=pos[1])

    @property
    def done(self) -> bool:
        """
        Checks if the exploration task is done for all robots.

        Returns:
            bool: True if the task is done for all robots, False otherwise.
        """
        for i, robot in enumerate(self._PROPS.robot_managers):
            if robot.is_done:
                self._set_goal(i, self._get_position(i))

            elif (self._PROPS.clock.clock - self._timeouts[i]).secs > Config.Robot.TIMEOUT:
                self._set_position(i, self._get_position(i))

        return False

    def _reset_timeout(self, index: int):
        """
        Resets the timeout for a specific robot.

        Args:
            index (int): The index of the robot.
        """
        self._timeouts[index] = self._PROPS.clock.clock
    
    def _set_role(self, index: int):
        """
        Sets the role for a specific robot.

        Args:
            index (int): The index of the robot.
        """
        self._roles[index] = self._config.ROLES[Config.General.RNG.integers(0, len(self._config.ROLES))]
        print(index, self._roles[index])
    
    def _get_position(self, index: int) -> PositionOrientation:
        """
        Returns a position for a specific robot depending on its role.

        Args:
            index (int): The index of the robot.
        """
        category = Config.General.RNG.choice(list(self._categories.keys() if self._roles[index].waypoints_in == "*" else self._roles[index].waypoints_in))

        zone_id = Config.General.RNG.choice(self._categories[category])

        position = getPositions(self._config, self._PROPS, {zone_id: 1}, self._PROPS.robot_managers[index].safe_distance)[0]

        orientation = 2 * math.pi * Config.General.RNG.random()

        return PositionOrientation(*position, orientation)

    def _set_position(self, index: int, position: PositionOrientation):
        """
        Sets the position of a specific robot and resets the timeout.

        Args:
            index (int): The index of the robot.
            position (PositionOrientation): The new position of the robot.
        """
        self._reset_timeout(index)
        self._PROPS.robot_managers[index].reset(position, None)

    def _set_goal(self, index: int, position: PositionOrientation):
        """
        Sets the goal position of a specific robot and resets the timeout.

        Args:
            index (int): The index of the robot.
            position (PositionOrientation): The new goal position of the robot.
        """
        self._reset_timeout(index)
        self._PROPS.robot_managers[index].reset(None, position)
