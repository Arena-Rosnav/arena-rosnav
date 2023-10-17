#! /usr/bin/env python3

import os
import traceback
from typing import Dict, List
from rospkg import RosPack
import rospkg
import yaml
import rospy

from std_msgs.msg import Int16, Empty as EmptyMsg
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from task_generator.manager.robot_manager import RobotManager
from task_generator.shared import ModelWrapper, Robot
from task_generator.simulators.base_simulator import BaseSimulator

from task_generator.tasks import TaskFactory, BaseTask

from task_generator.utils import ModelLoader, Utils
from task_generator.constants import Constants

from task_generator.simulators.simulator_factory import SimulatorFactory
from task_generator.simulators.gazebo_simulator import GazeboSimulator  # noqa
from task_generator.simulators.flatland_simulator import FlatlandSimulator  # noqa
from task_generator.simulators.simulator_factory import SimulatorFactory

from task_generator.manager.map_manager import MapManager
from task_generator.manager.obstacle_manager import ObstacleManager
from task_generator.manager.dynamic_manager.dynamic_manager import DynamicManager
from task_generator.manager.dynamic_manager.pedsim_manager import PedsimManager
from task_generator.manager.dynamic_manager.sfm_manager import SFMManager

from map_distance_server.srv import GetDistanceMap


def create_default_robot_list(robot_model: ModelWrapper, name: str, namespace: str, planner: str, agent: str) -> List[Robot]:
    return [Robot(
        model=robot_model,
        planner=planner,
        namespace=namespace,
        agent=agent,
        position=(0, 0, 0),
        name=name,
        record_data=False,
        extra=dict()
    )]

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


class TaskGenerator:
    """
    Task Generator Node
    Will initialize and reset all tasks. The task to use is read from the `/task_mode` param.
    """

    _task_mode: Constants.TaskMode
    _social_mode: Constants.SocialMode
    _auto_reset: bool

    _namespace: str
    _env_wrapper: BaseSimulator
    _task: BaseTask

    _pub_scenario_reset: rospy.Publisher
    _pub_scenario_finished: rospy.Publisher

    _start_time: float
    _number_of_resets: int
    _desired_resets: int

    def __init__(self) -> None:

        self._namespace = "/"

        # Params
        self._task_mode = Constants.TaskMode(
            rospy.get_param(os.path.join(self._namespace, "task_mode")))
        self._social_mode = Constants.SocialMode(
            rospy.get_param(os.path.join(self._namespace, "social_mode"), "pedsim"))
        self._auto_reset = bool(rospy.get_param("~auto_reset", True))

        # Publishers
        self._pub_scenario_reset = rospy.Publisher(
            "scenario_reset", Int16, queue_size=1)
        self._pub_scenario_finished = rospy.Publisher(
            'scenario_finished', EmptyMsg, queue_size=10)

        # Services
        rospy.Service("reset_task", Empty, self._reset_task_srv_callback)

        # Vars
        self._env_wrapper = SimulatorFactory.instantiate(Utils.get_simulator())("")

        rospy.loginfo(f"Launching task mode: {self._task_mode}")

        # Loaders
        self._robot_loader = ModelLoader(os.path.join(
            RosPack().get_path("arena-simulation-setup"), "robot"))

        self._start_time = rospy.get_time()
        self._task = self._get_predefined_task()
        rospy.set_param("/robot_names", self._task.robot_names)

        self._number_of_resets = 0
        self._desired_resets = int(str(rospy.get_param("desired_resets", 2)))

        self.srv_start_model_visualization = rospy.ServiceProxy(
            "start_model_visualization", Empty
        )
        self.srv_start_model_visualization(EmptyRequest())

        # rospy.sleep(5)

        self.reset_task(first_map=True)

        rospy.sleep(2)

        try:
            rospy.set_param("task_generator_setup_finished", True)
            self.srv_setup_finished = rospy.ServiceProxy(
                "task_generator_setup_finished", Empty
            )
            self.srv_setup_finished(EmptyRequest())
        except:
            pass

        self._number_of_resets = 0

        # The second reset below caused bugs and did not help according to my testing
        # self.reset_task()

        # Timers
        rospy.Timer(rospy.Duration(nsecs=int(0.5e9)), self._check_task_status)

    #SETUP

    def _get_predefined_task(self, **kwargs):
        """
        Gets the task based on the passed mode
        """
        if self._env_wrapper is None:
            self._env_wrapper = SimulatorFactory.instantiate(
                Utils.get_simulator())(self._namespace)

        rospy.wait_for_service("/distance_map")

        service_client_get_map = rospy.ServiceProxy(
            "/distance_map", GetDistanceMap)

        map_response = service_client_get_map()
        map_manager = MapManager(map_response)

        dynamic_manager: DynamicManager

        if self._social_mode == Constants.SocialMode.SFM:
            dynamic_manager = SFMManager(namespace="", simulator=self._env_wrapper)
        elif self._social_mode == Constants.SocialMode.PEDSIM:
            dynamic_manager = PedsimManager(namespace="", simulator=self._env_wrapper)
        else:
            dynamic_manager = DynamicManager(
                namespace=self._namespace, simulator=self._env_wrapper)

        obstacle_manager = ObstacleManager(
            namespace=self._namespace, map_manager=map_manager, simulator=self._env_wrapper, dynamic_manager=dynamic_manager)

        robot_managers = self._create_robot_managers()

        # For every robot
        # - Create a unique namespace name
        # - Create a robot manager
        # - Launch the robot.launch file

        rospy.logdebug("utils calls task factory")
        task = TaskFactory.instantiate(self._task_mode)(
            obstacle_manager=obstacle_manager,
            robot_managers=robot_managers,
            map_manager=map_manager,
            namespace=self._namespace,
            **kwargs
        )

        return task

    def _create_robot_managers(self) -> List[RobotManager]:
        # Read robot setup file
        robot_setup_file: str = str(rospy.get_param('/robot_setup_file', ""))

        robot_model: str = str(rospy.get_param("/model"))

        if robot_setup_file == "":
            robots = create_default_robot_list(
                robot_model=self._robot_loader.bind(robot_model),
                planner=str(rospy.get_param("/local_planner", "")),
                agent=str(rospy.get_param("/agent_name", "")),
                name=robot_model,
                namespace=os.path.join(self._namespace, robot_model),
            )
        else:
            robots = [
                Robot(
                    name=f"""{robot["model"]}_{i}_{robot.get("amount", 1)}""",
                    namespace=os.path.join(self._namespace, f"""{robot["model"]}_{i}_{robot.get("amount", 1)}"""),
                    planner=robot["planner"],
                    agent=robot["agent"],
                    record_data=False,
                    position=(0, 0, 0),
                    model=self._robot_loader.bind(robot["model"]),
                    extra=dict()
                )
                for robot in read_robot_setup_file(robot_setup_file)
                for i in range(robot.get("amount", 1))
            ]

        if Utils.get_arena_type() == Constants.ArenaType.TRAINING:
            return [RobotManager(simulator=self._env_wrapper, robot=robots[0])]

        robot_managers: List[RobotManager] = []

        for robot in robots:
            robot_managers.append(
                # RobotManager(os.path.join(namespace, name), simulator, robot)

                # old but working due to namespace issue with "/" prefix in robot name
                RobotManager(simulator=self._env_wrapper, robot=robot)
            )

        return robot_managers


    # RUNTIME

    def reset_task(self, **kwargs):
        self._start_time = rospy.get_time()

        self._env_wrapper.before_reset_task()

        rospy.loginfo("resetting")

        is_end = self._task.reset(callback=lambda:False, **kwargs)

        self._pub_scenario_reset.publish(self._number_of_resets)
        self._send_end_message_on_end(is_end)

        self._env_wrapper.after_reset_task()

        rospy.loginfo("=============")
        rospy.loginfo("Task Reseted!")
        rospy.loginfo("=============")

        self._number_of_resets += 1

    def _check_task_status(self, *args, **kwargs):
        if self._task.is_done:
            self.reset_task()

    def _reset_task_srv_callback(self, req: Empty):
        rospy.logdebug("Task Generator received task-reset request!")

        self.reset_task()

        return EmptyResponse()

    def _send_end_message_on_end(self, is_end: bool):
        if (
            (not is_end and self._task_mode == Constants.TaskMode.SCENARIO)
            or (self._task_mode != Constants.TaskMode.SCENARIO and self._number_of_resets < self._desired_resets)
        ):
            return

        rospy.loginfo("Shutting down. All tasks completed")

        # Send Task finished to Backend
        if rospy.get_param(os.path.join(self._namespace, "is_webapp_docker"), False):
            while self._pub_scenario_finished.get_num_connections() <= 0:
                pass

            self._pub_scenario_finished.publish(EmptyMsg())

        rospy.signal_shutdown("Finished all episodes of the current scenario")


if __name__ == "__main__":
    rospy.init_node("task_generator")

    task_generator = TaskGenerator()

    rospy.spin()


