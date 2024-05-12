import rospy
from dynamic_reconfigure.client import Client
from rosnav.node import MBFRosnavNode
from task_generator.shared import Namespace

from ...constants import DMRC_SERVER, DMRC_SERVER_ACTION, MBF_COMPATIBLE_TYPE
from .mbf_base import MBFLocalPlanner


class MBFRosnavLocalPlanner(MBFLocalPlanner):
    def __init__(
        self, ns: Namespace, name: str, planner: str, config: dict, *args, **kwargs
    ):
        self._ns = ns
        self._name = name
        self._planner = planner
        self._config = config if config is not None else {}

        self._active_planner = ""
        self._agent = MBFRosnavNode(ns=ns, agent_name=name, use_cuda=config["cuda"])

        self._dmrc_action = Client(
            self._ns(DMRC_SERVER_ACTION),
            config_callback=self._reconfigure,
        )

    def _reconfigure(self, config: dict):
        self._active_planner = config["base_local_planner"]
        self.deactivate()

    def activate(self):
        if self._active_planner == self.name:
            return

        rospy.loginfo(f"[LOCAL PLANNER] Activating '{self.name}' [{self.planner}]...")
        self._dmrc_action.update_configuration({"base_local_planner": self._name})
        # self._dmrc_params.update_configuration(self._config)

    def deactivate(self):
        pass
