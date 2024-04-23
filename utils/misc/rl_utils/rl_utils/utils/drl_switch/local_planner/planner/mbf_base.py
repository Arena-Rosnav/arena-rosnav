import rospy
from dynamic_reconfigure.client import Client
from rospy import loginfo_throttle
from std_srvs.srv import Empty
from task_generator.shared import Namespace

from ...constants import DMRC_SERVER, DMRC_SERVER_ACTION, MBF_COMPATIBLE_TYPE


class MBFLocalPlanner:
    def __init__(
        self,
        ns: Namespace,
        name: str,
        planner: MBF_COMPATIBLE_TYPE.LOCAL,
        config: dict,
        *args,
        **kwargs,
    ):
        self._ns = ns
        self._name = name
        self._planner = planner
        self._config = config if config is not None else {}

        clear_costmap_srv = self._ns("move_base_flex")("clear_costmaps")
        self._clear_costmap_service = rospy.ServiceProxy(clear_costmap_srv, Empty)

        self._dmrc_params = Client(
            self._ns(DMRC_SERVER)(self._planner.value),
            # config_callback=self._reconfigure,
        )
        self._dmrc_action = Client(
            self._ns(DMRC_SERVER_ACTION),
            config_callback=self._reconfigure,
        )

        self._active_planner = ""

    @property
    def name(self):
        return f"{self._planner.name} - {self._planner.value}: {self._name}"

    @property
    def planner(self):
        return self._planner.value

    def __str__(self):
        return self.planner

    def _reconfigure(self, config: dict):
        self._active_planner = config["base_local_planner"]
        self.deactivate()

    def activate(self):
        loginfo_throttle(1, f"[LOCAL PLANNER] Activating '{self.name}'...")

        clear_costmaps = self._active_planner != self.planner

        self._dmrc_action.update_configuration({"base_local_planner": self.planner})
        self._dmrc_params.update_configuration(self._config)

        if clear_costmaps:
            self._clear_costmap_service()

    def deactivate(self):
        pass
