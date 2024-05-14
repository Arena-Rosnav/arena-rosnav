import rospy
from dynamic_reconfigure.client import Client
from rospy import loginfo_throttle
from std_srvs.srv import Empty
from task_generator.shared import Namespace

from ...constants import DMRC_SERVER, DMRC_SERVER_ACTION, MBF_COMPATIBLE_TYPE


class MBFLocalPlanner:
    """
    MBFLocalPlanner class represents a local planner for MBF (Move Base Flex).
    It provides methods to activate and deactivate the local planner.

    Args:
        ns (Namespace): The namespace for the local planner.
        name (str): The name of the local planner.
        planner (MBF_COMPATIBLE_TYPE.LOCAL): The type of the local planner.
        config (dict): Configuration parameters for the local planner.
        *args: Additional positional arguments.
        **kwargs: Additional keyword arguments.
    """

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

        # Dynamic reconfigure clients
        # dmrc_mblegacy: Client for move_base_legacy_relay - contains planner specific parameters
        # dmrc_mbf: Client for move_base_flex - contains the active local planner
        self._dmrc_mblegacy = Client(
            self._ns(DMRC_SERVER)(self._planner.value),
            config_callback=self._mblegay_reconfigure,
        )
        self._dmrc_mbf = Client(
            self._ns(DMRC_SERVER_ACTION),
            config_callback=self._mbf_reconfigure,
        )

        self._active_planner = ""

    @property
    def name(self):
        """
        Get the name of the local planner.

        Returns:
            str: The name of the local planner.
        """
        return f"{self._planner.name} - {self._planner.value}: {self._name}"

    @property
    def planner(self):
        """
        Get the type of the local planner.

        Returns:
            MBF_COMPATIBLE_TYPE.LOCAL: The type of the local planner.
        """
        return self._planner.value

    def __str__(self):
        """
        Get a string representation of the local planner.

        Returns:
            str: A string representation of the local planner.
        """
        return self.planner

    def _mbf_reconfigure(self, config: dict):
        """
        Callback function for reconfiguring the local planner.

        Args:
            config (dict): The new configuration parameters for the local planner.
        """
        self._active_planner = config["base_local_planner"]
        self.deactivate()

    def _mblegay_reconfigure(self, config: dict):
        """
        Callback function for reconfiguring the local planner.

        Args:
            config (dict): The new configuration parameters for the local planner.
        """
        pass

    def activate(self):
        """
        Activate the local planner.
        """
        if self._active_planner == self.planner:
            return
        loginfo_throttle(1, f"[LOCAL PLANNER] Activating '{self.name}'...")

        self._dmrc_mbf.update_configuration({"base_local_planner": self.planner})
        self._dmrc_mblegacy.update_configuration(self._config)

    def deactivate(self):
        """
        Deactivate the local planner.
        """
        pass

    def close(self):
        pass
