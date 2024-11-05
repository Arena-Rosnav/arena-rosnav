from dataclasses import fields, is_dataclass
from typing import Any, Dict, TYPE_CHECKING

import rospy
from rl_utils.topic import Namespace

if TYPE_CHECKING:
    from .container import SimulationStateContainer


class StateDistributor:
    """Dynamically distributes dataclass states to ROS parameters maintaining hierarchy."""

    def __init__(
        self, container: "SimulationStateContainer", namespace: Namespace = None
    ) -> None:
        self._container = container
        self._base_namespace = (
            namespace("simulation_state_container")
            if namespace
            else "simulation_state_container"
        )
        self._param_cache: Dict[str, Any] = {}

    def distribute(self) -> None:
        """Distribute all states to ROS parameters."""
        self._distribute_recursive(self._container, self._base_namespace)

    def _distribute_recursive(self, obj: Any, namespace: str) -> None:
        """
        Recursively distribute nested dataclass structure to ROS parameters.

        Args:
            obj: Current object being processed
            namespace: Current parameter namespace
        """
        if not is_dataclass(obj):
            return

        for field in fields(obj):
            value = getattr(obj, field.name)
            param_path = f"{namespace}/{field.name}".lower()

            if is_dataclass(value):
                # Recurse into nested dataclass
                self._distribute_recursive(value, param_path)
            else:
                # Set parameter and cache value
                self._set_param(param_path, value)

    def _set_param(self, param_path: str, value: Any) -> None:
        """Set ROS parameter and cache the value."""
        rospy.set_param(param_path, value)
        self._param_cache[param_path] = value

    def get_param(self, param_path: str) -> Any:
        """Get parameter value from cache or ROS parameter server."""
        if param_path in self._param_cache:
            return self._param_cache[param_path]
        return rospy.get_param(param_path)

    def update_param(self, param_path: str, value: Any) -> None:
        """Update parameter value and cache."""
        self._set_param(param_path, value)

    def get_full_param_path(self, *path_parts: str) -> str:
        """Construct full parameter path from parts."""
        return "/".join([self._base_namespace, *path_parts])
