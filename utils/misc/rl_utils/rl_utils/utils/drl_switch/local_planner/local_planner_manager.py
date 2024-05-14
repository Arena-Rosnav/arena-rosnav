from typing import List, Union
from .planner.mbf_base import MBFLocalPlanner
from .planner.mbf_rosnav import MBFRosnavLocalPlanner
from ..constants import DMRC_SERVER, DMRC_SERVER_ACTION, MBF_COMPATIBLE_TYPE

from task_generator.shared import Namespace

import rospy
from std_srvs.srv import Empty

import time


class LocalPlannerManager:
    """
    A class that manages multiple local planners.

    Args:
        ns (Namespace): The namespace for the local planner.
        config (dict): The configuration for the local planner.

    Attributes:
        _ns (Namespace): The namespace for the local planner.
        _config (dict): The configuration for the local planner.
        _planners (list): A list of MBFLocalPlanner instances.

    Methods:
        __getitem__(self, index): Get the MBFLocalPlanner instance at the specified index.
        _setup_planners(self): Set up the MBFLocalPlanner instances based on the configuration.
        num_planners(self): Get the number of MBFLocalPlanner instances.
        activate(self, index): Activate the MBFLocalPlanner instance at the specified index.
    """

    def __init__(self, ns: Namespace, config: dict) -> None:
        self._ns = ns
        self._config = config
        self._train_mode = rospy.get_param("/train_mode", False)

        if self._train_mode:
            service_name_step = self._ns.simulation_ns("step_world")
            self._step_world_srv = rospy.ServiceProxy(
                service_name_step, Empty, persistent=True
            )
        self._setup_planners()

    def __getitem__(self, index: int) -> MBFLocalPlanner:
        """
        Get the MBFLocalPlanner instance at the specified index.

        Args:
            index (int): The index of the MBFLocalPlanner instance to get.

        Returns:
            MBFLocalPlanner: The MBFLocalPlanner instance at the specified index.
        """
        return self._planners[index]

    def _setup_planners(self) -> None:
        """
        Set up the MBFLocalPlanner instances based on the configuration.
        """
        if self._train_mode:
            # step the world to complete MBF initialization
            mbf_server_service = self._ns.simulation_ns(DMRC_SERVER)("set_parameters")
            mbf_action_service = self._ns.simulation_ns(DMRC_SERVER_ACTION)(
                "set_parameters"
            )
            for _ in range(10):
                time.sleep(0.1)
                self._step_world_srv()

                try:
                    rospy.wait_for_service(mbf_server_service, timeout=0.1)
                    rospy.wait_for_service(mbf_action_service, timeout=0.1)
                except rospy.ROSException:
                    continue

        self._planners: List[MBFLocalPlanner] = []
        for planner_config in self._config:
            if not planner_config["name"] in MBF_COMPATIBLE_TYPE.LOCAL.__members__:
                rospy.logfatal(
                    f"Local planner '{planner_config['name']}' not supported."
                )
                rospy.signal_shutdown("Local planner not supported.")
                raise ValueError(
                    f"Local planner '{planner_config['name']}' not supported."
                )

            self._planners.append(
                LocalPlannerManager.initialize_local_planner(self._ns, planner_config)
            )

    @property
    def num_planners(self) -> int:
        """
        Get the number of MBFLocalPlanner instances.

        Returns:
            int: The number of MBFLocalPlanner instances.
        """
        return len(self._planners)

    def activate(self, index: int) -> None:
        """
        Activate the MBFLocalPlanner instance at the specified index.

        Args:
            index (int): The index of the MBFLocalPlanner instance to activate.
        """
        self[index].activate()

    def close(self) -> None:
        """
        Close the LocalPlannerManager.
        """
        for planner in self._planners:
            planner.close()

    @staticmethod
    def initialize_local_planner(ns: Namespace, config: dict) -> MBFLocalPlanner:
        if config["name"] == "rosnav":
            return MBFRosnavLocalPlanner(
                ns=ns,
                name=config["config"]["agent"],
                planner=MBF_COMPATIBLE_TYPE.LOCAL[config["name"]],
                config=config["config"],
            )

        return MBFLocalPlanner(
            ns=ns,
            name=(config["alias"] if "alias" in config else config["name"]),
            planner=MBF_COMPATIBLE_TYPE.LOCAL[config["name"]],
            config=config["config"],
        )
