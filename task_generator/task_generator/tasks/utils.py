import os
import traceback

import rospkg
import rospy
import yaml
from map_distance_server.srv import GetDistanceMap
from task_generator.constants import Constants
from task_generator.manager.map_manager import MapManager
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.simulators.base_simulator import BaseSimulator
from task_generator.simulators.flatland_simulator import FlatlandRandomModel
from task_generator.simulators.gazebo_simulator import GazeboSimulator
from task_generator.simulators.simulator_factory import SimulatorFactory
from task_generator.tasks.dynamic_map_random import DynamicMapRandomTask
from task_generator.tasks.dynamic_map_staged import DynamicMapStagedRandomTask
from task_generator.tasks.random import RandomTask
from task_generator.tasks.scenario import ScenarioTask
from task_generator.tasks.staged import StagedRandomTask
from task_generator.tasks.task_factory import TaskFactory
from task_generator.utils import Utils


def get_predefined_task(
    namespace: str, mode: str, simulator: BaseSimulator = None, **kwargs
):
    """
    Gets the task based on the passed mode
    """
    if simulator is None:
        simulator = SimulatorFactory.instantiate(Utils.get_simulator())(namespace)

    rospy.wait_for_service("/distance_map")

    service_client_get_map = rospy.ServiceProxy("/distance_map", GetDistanceMap)

    map_response = service_client_get_map()
    map_manager = MapManager(map_response)

    simulator.map_manager = map_manager

    obstacle_manager = ObstacleManager(
        namespace, map_manager, simulator, rospy.get_param("/reset_remove_all", True)
    )
    robot_managers = create_robot_managers(namespace, map_manager, simulator)

    # For every robot
    # - Create a unique namespace name
    # - Create a robot manager
    # - Launch the robot.launch file

    return TaskFactory.instantiate(
        mode,
        obstacle_manager,
        robot_managers,
        map_manager,
        namespace=namespace,
        **kwargs,
    )


def create_robot_managers(
    namespace: str, map_manager: MapManager, simulator: BaseSimulator
):
    # Read robot setup file
    robot_setup_file = rospy.get_param("/robot_setup_file", "")

    if robot_setup_file == "":
        robots = create_default_robot_list(
            rospy.get_param("/model"),
            rospy.get_param("/local_planner", ""),
            rospy.get_param("/agent_name", ""),
        )
    else:
        robots = read_robot_setup_file(robot_setup_file)

    if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
        return [RobotManager(namespace, map_manager, simulator, robots[0])]

    robot_managers = []

    for robot in robots:
        amount = robot["amount"]

        for r in range(0, amount):
            name = f"{robot['model']}_{r}_{len(robot_managers)}"

            robot_managers.append(
                RobotManager(namespace + "/" + name, map_manager, simulator, robot)
            )

    return robot_managers


def read_robot_setup_file(setup_file):
    try:
        with open(
            os.path.join(
                rospkg.RosPack().get_path("task_generator"), "robot_setup", setup_file
            ),
            "r",
        ) as file:
            return yaml.safe_load(file)["robots"]
    except:
        traceback.print_exc()
        rospy.signal_shutdown()


def create_default_robot_list(robot_model, planner, agent):
    return [{"model": robot_model, "planner": planner, "agent": agent, "amount": 1}]
