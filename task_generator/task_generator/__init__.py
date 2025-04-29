import typing

import launch
import rclpy
import rclpy.node
import rclpy.impl.rcutils_logger

from .utils.ros_params import ROSParamServer


class NodeInterface:
    class Taskgen_T(ROSParamServer, rclpy.node.Node):
        ...

    node: Taskgen_T

    def __init__(self) -> None:
        ...

    @property
    def _logger(self) -> rclpy.impl.rcutils_logger.RcutilsLogger:
        return self.node.get_logger().get_child(type(self).__name__)


def init_task_gen_node(
    do_launch: typing.Callable[[launch.LaunchDescription], None],
) -> ROSParamServer:

    from .node import TaskGenerator
    NodeInterface.node = TaskGenerator(do_launch=do_launch)

    # TODO deprecate
    from .shared import configure_node
    configure_node(NodeInterface.node)

    return NodeInterface.node
