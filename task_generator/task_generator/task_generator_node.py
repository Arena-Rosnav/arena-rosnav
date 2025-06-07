#! /usr/bin/env python3
import asyncio
import multiprocessing
import queue
import traceback
import typing

import launch
import rclpy
import rclpy.executors


def init_launch_service(
    CONCURRENT: bool
) -> tuple[
    typing.Callable[[], typing.Any],
    typing.Callable[[launch.LaunchDescription], typing.Any],
    typing.Callable[[], None]
]:
    """
    Initiate launch service.
    Args:
        CONCURRENT: is node run concurrently in python thread?
    Returns:
        A tuple consisting of:
            loop function to call after `executor.spin_once()`,
            function that accepts a `launch.LaunchDescription` and schedules its launch.
            cleanup function to call at shutdown
    """

    def _do_launch(
            launch_description: launch.LaunchDescription
    ) -> typing.Callable[[], typing.Any]:

        # https://github.com/ros2/launch/issues/724#issue-1851039469
        def run_process(stop_event, launch_description):

            # https://github.com/ros2/launch/issues/724#issuecomment-1829050299
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            launch_service = launch.launch_service.LaunchService()
            launch_service.include_launch_description(launch_description)
            launch_task = loop.create_task(launch_service.run_async())

            try:
                loop.run_until_complete(
                    loop.run_in_executor(
                        None,
                        stop_event.wait
                    )
                )
            except KeyboardInterrupt:
                stop_event.set()

            if not launch_task.done():
                asyncio.ensure_future(launch_service.shutdown(), loop=loop)
                try:
                    loop.run_until_complete(launch_task)
                except KeyboardInterrupt:
                    stop_event.set()

        stop_event = multiprocessing.Event()
        process = multiprocessing.Process(
            target=run_process, args=(
                stop_event,
                launch_description
            ),
            daemon=True
        )
        process.start()

        def shutdown():
            stop_event.set()

        return shutdown

    if CONCURRENT:
        launch_queue = queue.Queue[launch.LaunchDescription]()
        shutdown_queue = queue.Queue[typing.Callable[[], typing.Any]]()

        def process_queue() -> None:
            try:
                launch_description = launch_queue.get(False)
            except queue.Empty:
                return None

            shutdown_queue.put(_do_launch(launch_description))
            return process_queue()

        def launch_soon(launch_description: launch.LaunchDescription) -> None:
            launch_queue.put(launch_description)

        def cleanup():
            while not shutdown_queue.empty():
                do_shutdown = shutdown_queue.get(True)
                do_shutdown()

        return process_queue, launch_soon, cleanup

    def _noop() -> None:
        return None

    return _noop, _do_launch, _noop


def main(args=None):
    rclpy.init()

    CONCURRENT = True

    if CONCURRENT:
        executor = rclpy.executors.MultiThreadedExecutor()
    else:
        executor = rclpy.executors.SingleThreadedExecutor()

    launch_loop, do_launch, launch_cleanup = init_launch_service(
        CONCURRENT=CONCURRENT
    )

    from . import NodeInterface

    node = NodeInterface.init_task_gen_node(do_launch=do_launch)

    executor.add_node(node)

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        while rclpy.ok():
            executor.spin_once()
            launch_loop()
    except KeyboardInterrupt:
        node.get_logger().info(traceback.format_exc())
        node.get_logger().info('Keyboard interrupt, shutting down.')

    launch_cleanup()
    node.destroy_node()
    rclpy.try_shutdown()
