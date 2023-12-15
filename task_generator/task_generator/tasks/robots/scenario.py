from typing import List, NamedTuple
import rospy
from task_generator.constants import Constants
from task_generator.shared import PositionOrientation, PositionRadius, rosparam_get
from task_generator.tasks.robots import TM_Robots
from task_generator.tasks.task_factory import TaskFactory


class RobotGoal(NamedTuple):
    start: PositionOrientation
    goal: PositionOrientation

    @staticmethod
    def parse(obj: dict) -> "RobotGoal":
        return RobotGoal(
            start = PositionOrientation(*obj.get("start", [])),
            goal = PositionOrientation(*obj.get("goal", []))
        )

@TaskFactory.register_robots(Constants.TaskMode.TM_Robots.SCENARIO)
class TM_Scenario(TM_Robots):
    def reset(self, **kwargs):
        SCENARIO_ROBOTS: List[RobotGoal] = [
            RobotGoal.parse(robot)
            for robot
            in rosparam_get(list, "scenario/robots", [])
        ]

        # check robot manager length
        managed_robots = self._PROPS.robot_managers

        scenario_robots_length = len(SCENARIO_ROBOTS)
        setup_robot_length = len(managed_robots)

        if setup_robot_length > scenario_robots_length:
            managed_robots = managed_robots[:scenario_robots_length]
            rospy.logwarn_once("Roboto setup contains more robots than the scenario file.")

        if scenario_robots_length > setup_robot_length:
            SCENARIO_ROBOTS = SCENARIO_ROBOTS[:setup_robot_length]
            rospy.logwarn_once("Scenario file contains more robots than setup.")

        for (robot, config) in zip(managed_robots, SCENARIO_ROBOTS):
            robot.reset(
                start_pos=config.start,
                goal_pos=config.goal
            )
            self._PROPS.world_manager.forbid([
                PositionRadius(
                    x=config.start.x,
                    y=config.start.y,
                    radius=robot.safe_distance
                ),
                PositionRadius(
                    x=config.goal.x,
                    y=config.goal.y,
                    radius=robot.safe_distance
                ),
            ])