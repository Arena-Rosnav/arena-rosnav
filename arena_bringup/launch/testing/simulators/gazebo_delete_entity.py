#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import DeleteEntity  # Import the correct service type
import sys

class DeleteEntityClient(Node):
    def __init__(self):
        super().__init__('delete_entity_client')
        self.client = self.create_client(DeleteEntity, '/world/default/remove')  # Correct service name
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info('Service is now available.')

    def delete_entity(self, entity_name):
        request = DeleteEntity.Request()
        request.name = entity_name  # Set the entity name in the request
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f"Deleted entity: {entity_name}")
            else:
                self.get_logger().error(f"Failed to delete entity: {entity_name}. Reason: {future.result().reason}")
        else:
            self.get_logger().error("Service call failed")

def main(args=None):
    if args is None:
        args = sys.argv[1:]  # Use sys.argv to get command line arguments

    rclpy.init(args=args)
    entity_name = args[0] if args else 'box_0'  # Default to 'box_0' if no argument provided
    node = DeleteEntityClient()
    node.delete_entity(entity_name)
    rclpy.shutdown()

if __name__ == '__main__':
    main()