from .utils.ros_params import ROSParamServer
import rclpy

TASKGEN_NODE: ROSParamServer


class NodeInterface:
    node: ROSParamServer

    def __init__(self) -> None:
        ...


def init_task_gen_node(args=None):
    global TASKGEN_NODE

    from .node import TaskGenerator
    NodeInterface.node = TaskGenerator()

    from .shared import configure_node
    configure_node(NodeInterface.node)

    NodeInterface.node.post_init()

    while True:
        # rclpy.spin_once(TASKGEN_CONFIG_NODE)
        rclpy.spin_once(NodeInterface.node)
