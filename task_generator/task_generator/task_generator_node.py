#! /usr/bin/env python3
import rclpy


def main(args=None):
    rclpy.init()
    from . import init_task_gen_node

    node = init_task_gen_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
