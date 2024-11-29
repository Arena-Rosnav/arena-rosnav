#! /usr/bin/env python3
import rclpy


def main(args=None):
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor()

    from . import init_task_gen_node

    node = init_task_gen_node()

    executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
