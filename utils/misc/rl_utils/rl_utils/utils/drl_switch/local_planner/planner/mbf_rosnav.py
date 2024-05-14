from multiprocessing import Process
from rl_utils.utils.drl_switch.constants import MBF_COMPATIBLE_TYPE
import rospy
from rosnav.node import MBFRosnavNode
from task_generator.shared import Namespace

from .mbf_base import MBFLocalPlanner


class MBFRosnavLocalPlanner(MBFLocalPlanner):
    def __init__(
        self,
        ns: Namespace,
        name: str,
        planner: MBF_COMPATIBLE_TYPE.LOCAL,
        config: dict,
        *args,
        **kwargs,
    ):
        super().__init__(ns, name, planner, config, *args, **kwargs)
        self._process = None

        self._agent = MBFRosnavNode(
            ns=ns, agent_name=name, cuda_device=config["cuda_device"]
        )
        self.activate()

    def _mblegay_reconfigure(self, config: dict):
        self._active_planner = config["active_agent"]

    def _mbf_reconfigure(self, config: dict):
        pass

    def activate(self):
        if self._active_planner == self._name:
            return

        rospy.loginfo(f"[LOCAL PLANNER] Activating '{self.name}' ...")
        self._dmrc_mbf.update_configuration({"base_local_planner": self._planner.value})
        self._dmrc_mblegacy.update_configuration({"active_agent": self._name})

    def deactivate(self):
        pass

    def start_process(self):
        if self._process is not None:
            self.stop_process()

        self._process = Process(
            target=MBFRosnavLocalPlanner.run_agent,
            args=(self._ns, self._name, self._config),
        )
        self._process.start()

    def stop_process(self):
        if self._process is not None:
            self._process.terminate()
            self._process.join()
            self._process = None

    def close(self):
        pass
        # self.stop_process()

    @staticmethod
    def run_agent(ns: Namespace, name: str, config: dict, init_node: bool = True):
        if init_node:
            rospy.init_node(name, disable_signals=False)

        MBFRosnavNode(ns=ns, agent_name=name, cuda_device=config["cuda_device"])

        if init_node:
            rospy.spin()
