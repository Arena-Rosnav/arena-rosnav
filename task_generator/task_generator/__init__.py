from .utils.ros_params import ROSParamServer
from .constants.runtime import Configuration
import rclpy


class TaskgenNode(ROSParamServer, rclpy.node.Node):
    ...


TASKGEN_NODE: TaskgenNode


class NodeInterface:
    _node: TaskgenNode

    def __init__(self) -> None:
        from . import TASKGEN_NODE
        self._node = TASKGEN_NODE


def init_task_gen_node(args=None):
    global TASKGEN_NODE

    from .node import TaskGenerator

    TASKGEN_NODE = TaskGenerator()

    from .shared import configure_node
    configure_node(TASKGEN_NODE)

    TASKGEN_NODE.post_init()

    while True:
        # rclpy.spin_once(TASKGEN_CONFIG_NODE)
        rclpy.spin_once(TASKGEN_NODE)
