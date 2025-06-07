import typing

import launch
import rclpy
import rclpy.node
import rclpy.client
import rclpy.callback_groups
import rclpy.impl.rcutils_logger
from arena_rclpy_mixins import ArenaMixinNode
from arena_rclpy_mixins.ROSParamServer import ROSParamServer


class SafeCallbackNode(rclpy.node.Node):
    """
    Automatically make clients part of a new MutuallyExclusiveCallbackGroup to avoid deadlocks.
    """

    def create_client(self, *args, callback_group: rclpy.callback_groups.CallbackGroup | None = None, **kwargs) -> rclpy.client.Client:
        if callback_group is None:
            callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        return super().create_client(*args, callback_group=callback_group, **kwargs)


class NodeInterface:
    class Taskgen_T(ArenaMixinNode, SafeCallbackNode):
        ...

    node: Taskgen_T

    def __init__(self) -> None:
        ...

    @property
    def _logger(self) -> rclpy.impl.rcutils_logger.RcutilsLogger:
        return self.node.get_logger().get_child(type(self).__name__)

    @classmethod
    def init_task_gen_node(
        cls,
        do_launch: typing.Callable[[launch.LaunchDescription], None],
    ) -> ROSParamServer:

        from .node import TaskGenerator
        NodeInterface.node = TaskGenerator(do_launch=do_launch)

        # TODO deprecate
        from .shared import configure_node
        configure_node(NodeInterface.node)

        return NodeInterface.node
