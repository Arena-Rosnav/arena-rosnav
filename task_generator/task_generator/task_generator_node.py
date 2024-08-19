#! /usr/bin/env python3

from . import init_task_gen_node

def main(args=None):
    init_task_gen_node()
    from . import TASKGEN_NODE 
    TASKGEN_NODE.post_init()
    # Clean the node if necessary
    # task_generator.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()
