import rclpy.node

TASKGEN_CONFIG_NODE: rclpy.node.Node
TASKGEN_NODE: rclpy.node.Node

class NodeInterface:
    _node: rclpy.node.Node

    def __init__(self) -> None:
        from . import TASKGEN_NODE
        self._node = TASKGEN_NODE

class ConfigNodeInterface:
    _config_node: rclpy.node.Node

    def __init__(self) -> None:
        from . import TASKGEN_CONFIG_NODE
        self._config_node = TASKGEN_CONFIG_NODE

def init_task_gen_node(args=None):
    global TASKGEN_NODE, TASKGEN_CONFIG_NODE
    
    from .constants.runtime import TaskGenerator_ConfigNode
    from .node import TaskGenerator

    TASKGEN_CONFIG_NODE = TaskGenerator_ConfigNode()
    TASKGEN_NODE = TaskGenerator()