#! /usr/bin/env python3
import rclpy


def main(args=None):
    rclpy.init()
    from . import init_task_gen_node
    init_task_gen_node()
    from . import _TASKGEN_NODE, TASKGEN_CONFIG_NODE
    _TASKGEN_NODE.post_init()

    # Clean the node if necessary
    # task_generator.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
