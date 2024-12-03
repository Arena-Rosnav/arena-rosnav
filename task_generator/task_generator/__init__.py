import launch
import typing
from .utils.ros_params import ROSParamServer
import rclpy


class NodeInterface:
    class Taskgen_T(ROSParamServer, rclpy.node.Node):
        ...
    node: Taskgen_T

    def __init__(self) -> None:
        ...


def init_task_gen_node(
    do_launch: typing.Callable[[launch.LaunchDescription], None],
) -> ROSParamServer:

    from .node import TaskGenerator
    NodeInterface.node = TaskGenerator(do_launch=do_launch)

    # TODO deprecate
    from .shared import configure_node
    configure_node(NodeInterface.node)

    return NodeInterface.node
