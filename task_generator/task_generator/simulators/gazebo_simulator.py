import rclpy
from rclpy.node import Node

from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity, SetEntityPose, ControlWorld
from ros_gz_interfaces.msg import EntityFactory, WorldControl
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
import sys
from task_generator.simulators import SimulatorFactory
from task_generator.shared import rosparam_get
from task_generator.utils.geometry import quaternion_from_euler
from task_generator.constants import Constants
from task_generator.constants.runtime import Config
from task_generator.simulators import BaseSimulator

from task_generator.shared import ModelType, Namespace, PositionOrientation, RobotProps

class GazeboSimulator(BaseSimulator):
    def __init__(self, namespace, node: Node = None):
        """Initialize GazeboSimulator
        
        Args:
            namespace: Namespace for the simulator
            node (Node, optional): ROS2 node instance
        """
        # Ensure we have a valid node name
        node_name = namespace.strip('/').replace('/', '_')
        if not node_name:
            node_name = "gazebo_simulator"  # Default name if namespace is empty
            
        super().__init__(namespace=Namespace(node_name))
        
        # Store node reference
        self._node = node
        if self._node is None:
            from task_generator import TASKGEN_NODE
            self._node = TASKGEN_NODE
        
        self._goal_pub = self._node.create_publisher(
            PoseStamped,
            self._namespace("/goal"),
            10
        )
        self._node.declare_parameter('robot_model', '')
        self._robot_name = self._node.get_parameter('robot_model').value
        
        # Initialize service clients
        try:
            self._spawn_entity = self._node.create_client(SpawnEntity, '/world/diff_drive/create')
            self._delete_entity = self._node.create_client(DeleteEntity, '/world/diff_drive/remove')
            self._set_entity_pose = self._node.create_client(SetEntityPose, '/world/diff_drive/set_pose')
            self._control_world = self._node.create_client(ControlWorld, '/world/diff_drive/control')
        except Exception as e:
            self._node.get_logger().error(f"Error creating clients: {str(e)}")
            return

        self._node.get_logger().info("Waiting for gazebo services...")
        services = [
            (self._spawn_entity, "Spawn entity"),
            (self._delete_entity, "Delete entity"),
            (self._set_entity_pose, "Set entity pose"),
            (self._control_world, "Control world")
        ]

        for service, name in services:
            if not self._wait_for_service(service, name):
                return

        self._node.get_logger().info("All Gazebo services are available now.")

    def before_reset_task(self):
        self.pause_simulation()

    def after_reset_task(self):
        try:
            self.unpause_simulation()
        except Exception as e:  # Geändert von ServiceException zu Exception
            self._node.get_logger().warn(f"Service error: {e}")
            raise  # Re-raise the exception

    def _wait_for_service(self, service, name, timeout=15.0):
        timeout_loop = timeout
        while not service.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn(f"{name} service not available, waiting again...")
            timeout_loop -= 1.0
            if timeout_loop <= 0:
                self._node.get_logger().error(f"{name} service not available after {timeout} seconds")
                return False
        return True

    def move_entity(self, name, position):
        request = SetEntityPose.Request()
        request.entity = name
        request.pose = Pose(
            position=Point(x=position.x, y=position.y, z=0),
            orientation=Quaternion(*quaternion_from_euler(0.0, 0.0, position.orientation, axes="sxyz"))
        )
        future = self._set_entity_pose.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().success

    def spawn_entity(self, entity):
        request = SpawnEntity.Request()
        request.entity_factory = EntityFactory()
        request.entity_factory.name = entity.name
        request.entity_factory.type = 'sdf'  # or 'urdf' depending on your model type
        request.entity_factory.sdf = entity.model.get(self.MODEL_TYPES).description
        request.entity_factory.pose = Pose(
            position=Point(x=entity.position.x, y=entity.position.y, z=0),
            orientation=Quaternion(*quaternion_from_euler(0.0, 0.0, entity.position.orientation, axes="sxyz"))
        )

        if isinstance(entity, RobotProps):
            self._node.declare_parameter(
                Namespace(entity.name)("robot_description"), 
                entity.model.get(self.MODEL_TYPES).description
            )
            self._node.declare_parameter(
                Namespace(entity.name)("tf_prefix"), 
                str(Namespace(entity.name))
            )

        future = self._spawn_entity.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().success

    def delete_entity(self, name: str):
        request = DeleteEntity.Request()
        request.entity = name
        future = self._delete_entity.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().success
    
    def pause_simulation(self):
        request = ControlWorld.Request()
        request.world_control = WorldControl()
        request.world_control.pause = True
        future = self._control_world.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().success

    def unpause_simulation(self):
        request = ControlWorld.Request()
        request.world_control = WorldControl()
        request.world_control.pause = False
        future = self._control_world.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().success

    def step_simulation(self, steps):
        request = ControlWorld.Request()
        request.world_control = WorldControl()
        request.world_control.multi_step = steps
        future = self._control_world.call_async(request)
        rclpy.spin_until_future_complete(self._node, future)
        return future.result().success

    def _publish_goal(self, goal: PositionOrientation):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self._node.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal.x
        goal_msg.pose.position.y = goal.y
        goal_msg.pose.orientation = Quaternion(
            *quaternion_from_euler(0.0, 0.0, goal.orientation, axes="sxyz"))

        self._goal_pub.publish(goal_msg)