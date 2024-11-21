from .utils.ros_params import ROSParamServer
import rclpy


class NodeInterface:
    node: ROSParamServer

    def __init__(self) -> None:
        ...


def init_task_gen_node(args=None) -> ROSParamServer:

    from .node import TaskGenerator
    NodeInterface.node = TaskGenerator()

    from .shared import configure_node
    configure_node(NodeInterface.node)

    NodeInterface.node.post_init()

    return NodeInterface.node
