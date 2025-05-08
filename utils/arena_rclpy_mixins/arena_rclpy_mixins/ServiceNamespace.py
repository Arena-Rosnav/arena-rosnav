
import rclpy.node

from .shared import Namespace


class ServiceNamespace(rclpy.node.Node):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def service_namespace(self, *args: str) -> Namespace:
        """
        `rclpy.node.Node.create_service` doesn't utilize the node namespace (contrary to the doc). Use this to prefix service names until fixed.
        """
        return Namespace(self.get_fully_qualified_name())(*args)
