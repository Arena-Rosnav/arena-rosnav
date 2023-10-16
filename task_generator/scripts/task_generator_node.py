#! /usr/bin/env python3

import os
from rospkg import RosPack
import rospy
from std_msgs.msg import Int16, Empty as EmptyMsg
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from task_generator.simulators.base_simulator import BaseSimulator
from task_generator.tasks.base_task import BaseTask

from task_generator.utils import ModelLoader, Utils
from task_generator.constants import Constants

from task_generator.tasks.utils import get_predefined_task
from task_generator.simulators.simulator_factory import SimulatorFactory
from task_generator.simulators.gazebo_simulator import GazeboSimulator  # noqa
from task_generator.simulators.flatland_simulator import FlatlandSimulator  # noqa


class TaskGenerator:
    """
    Task Generator Node
    Will initialize and reset all tasks. The task to use is read from the `/task_mode` param.
    """

    _namespace: str

    _pub_scenario_reset: rospy.Publisher
    _pub_scenario_finished: rospy.Publisher

    _env_wrapper: BaseSimulator

    _start_time: float
    _task: BaseTask

    _number_of_resets: int
    _desired_resets: int

    def __init__(self) -> None:

        self._namespace = "/"

        # Params
        self.task_mode: Constants.TaskMode = Constants.TaskMode(
            rospy.get_param(os.path.join(self._namespace, "task_mode")))
        self.social_mode: Constants.SocialMode = Constants.SocialMode(
            rospy.get_param(os.path.join(self._namespace, "social_mode"), "pedsim"))
        self.auto_reset: bool = bool(rospy.get_param("~auto_reset", True))

        # Publishers
        self._pub_scenario_reset = rospy.Publisher(
            "scenario_reset", Int16, queue_size=1)
        self._pub_scenario_finished = rospy.Publisher(
            'scenario_finished', EmptyMsg, queue_size=10)

        # Services
        rospy.Service("reset_task", Empty, self._reset_task_srv_callback)

        # Vars
        self._env_wrapper = SimulatorFactory.instantiate(
            Utils.get_simulator())("")

        rospy.loginfo(f"Launching task mode: {self.task_mode}")

        # Loaders
        robot_loader = ModelLoader(os.path.join(
            RosPack().get_path("arena-simulation-setup"), "robot"))

        self._start_time = rospy.get_time()
        self._task = get_predefined_task(
            namespace="", mode=self.task_mode, simulator=self._env_wrapper, social_mode=self.social_mode, robot_loader=robot_loader)
        rospy.set_param("/robot_names", self._task.robot_names)

        self._number_of_resets = 0
        self._desired_resets = int(str(rospy.get_param("desired_resets", 2)))

        self.srv_start_model_visualization = rospy.ServiceProxy(
            "start_model_visualization", Empty)
        self.srv_start_model_visualization(EmptyRequest())

        rospy.sleep(5)

        self.reset_task()

        rospy.sleep(2)

        try:
            rospy.set_param("task_generator_setup_finished", True)
            self.srv_setup_finished = rospy.ServiceProxy(
                "task_generator_setup_finished", Empty)
            self.srv_setup_finished(EmptyRequest())
        except:
            pass

        self._number_of_resets = 0

        # The second reset below caused bugs and did not help according to my testing
        # self.reset_task()

        # Timers
        rospy.Timer(rospy.Duration(nsecs=int(0.5e9)), self._check_task_status)

    def reset_task(self):
        self._start_time = rospy.get_time()

        self._env_wrapper.before_reset_task()

        rospy.loginfo("resetting")

        is_end = self._task.reset()

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
            (not is_end and self.task_mode == Constants.TaskMode.SCENARIO)
            or (self.task_mode != Constants.TaskMode.SCENARIO and self._number_of_resets < self._desired_resets)
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
