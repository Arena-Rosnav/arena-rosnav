import traceback
from typing import Dict, List, Optional

import rospy
import rospkg
import yaml
import os
from task_generator.constants import Constants
from task_generator.shared import Model, ModelWrapper, Robot
from task_generator.simulators.base_simulator import BaseSimulator

from task_generator.simulators.simulator_factory import SimulatorFactory
from task_generator.manager.map_manager import MapManager
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.dynamic_manager.dynamic_manager import DynamicManager
from task_generator.manager.dynamic_manager.pedsim_manager import PedsimManager
from task_generator.manager.dynamic_manager.sfm_manager import SFMManager
from task_generator.manager.robot_manager import RobotManager
from task_generator.tasks.task_factory import TaskFactory
from task_generator.tasks.random import RandomTask  # noqa
from task_generator.tasks.scenario import ScenarioTask  # noqa
from task_generator.tasks.staged import StagedRandomTask  # noqa
from task_generator.tasks.random_scenario import RandomScenarioTask  # noqa
from task_generator.utils import ModelLoader, Utils

from map_distance_server.srv import GetDistanceMap

#TODO only task_generator_node uses all of this, turn this into its instance methods to simplify the calls

def get_predefined_task(namespace: str, mode: Constants.TaskMode, robot_loader: ModelLoader, simulator: Optional[BaseSimulator] = None, social_mode: Constants.SocialMode = Constants.SocialMode.PEDSIM, **kwargs):
    """
    Gets the task based on the passed mode
    """
    if simulator is None:
        simulator = SimulatorFactory.instantiate(
            Utils.get_simulator())(namespace)

    rospy.wait_for_service("/distance_map")

    service_client_get_map = rospy.ServiceProxy(
        "/distance_map", GetDistanceMap)

    map_response = service_client_get_map()

    map_manager = MapManager(map_response)

    dynamic_manager: DynamicManager

    if social_mode == Constants.SocialMode.SFM:
        dynamic_manager = SFMManager(namespace=namespace, simulator=simulator)
    elif social_mode == Constants.SocialMode.PEDSIM:
        dynamic_manager = PedsimManager(
            namespace=namespace, simulator=simulator)
    else:
        dynamic_manager = DynamicManager(
            namespace=namespace, simulator=simulator)

    obstacle_manager = ObstacleManager(
        namespace=namespace, map_manager=map_manager, simulator=simulator, dynamic_manager=dynamic_manager)

    robot_managers = create_robot_managers(namespace=namespace, simulator=simulator, robot_loader=robot_loader)

    # For every robot
    # - Create a unique namespace name
    # - Create a robot manager
    # - Launch the robot.launch file

    print("utils calls task factory")
    task = TaskFactory.instantiate(mode)(
        obstacle_manager=obstacle_manager,
        robot_managers=robot_managers,
        map_manager=map_manager,
        namespace=namespace,
        **kwargs
    )

    return task


def create_robot_managers(namespace: str, simulator: BaseSimulator, robot_loader: ModelLoader) -> List[RobotManager]:
    # Read robot setup file
    robot_setup_file: str = str(rospy.get_param('/robot_setup_file', ""))

    robot_model: str = str(rospy.get_param("/model"))

    if robot_setup_file == "":
        robots = create_default_robot_list(
            robot_model=robot_loader.bind(robot_model),
            planner=str(rospy.get_param("/local_planner", "")),
            agent=str(rospy.get_param("/agent_name", "")),
            name=robot_model,
            namespace=f"{namespace}/{robot_model}",
        )
    else:
        robots = [
            Robot(
                name=f"""{robot["model"]}_{i}_{robot.get("amount", 1)}""",
                namespace=f"""{namespace}/{robot["model"]}_{i}_{robot.get("amount", 1)}""",
                planner=robot["planner"],
                agent=robot["agent"],
                record_data=False,
                position=(0,0,0),
                model=robot_loader.bind(robot["model"]),
                extra=dict()
            )
            for robot in read_robot_setup_file(robot_setup_file)
            for i in range(robot.get("amount", 1))
        ]

    if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
        return [RobotManager(simulator=simulator, robot=robots[0])]

    robot_managers: List[RobotManager] = []

    for robot in robots:
        robot_managers.append(
            # RobotManager(os.path.join(namespace, name), simulator, robot)

            # old but working due to namespace issue with "/" prefix in robot name
            RobotManager(simulator=simulator, robot=robot)
        )

    return robot_managers


def read_robot_setup_file(setup_file: str) -> List[Dict]:
    try:
        with open(
            os.path.join(rospkg.RosPack().get_path(
                "task_generator"), "robot_setup", setup_file),
            "r"
        ) as f:
            robots: List[Dict] = yaml.safe_load(f)["robots"]

        return robots

    except:
        traceback.print_exc()
        rospy.signal_shutdown("")
        raise Exception()


def create_default_robot_list(robot_model: ModelWrapper, name:str, namespace: str, planner: str, agent: str) -> List[Robot]:
    return [Robot(
        model=robot_model,
        planner=planner,
        namespace=namespace,
        agent=agent,
        position=(0,0,0),
        name=name,
        record_data=False,
        extra=dict()
    )]
