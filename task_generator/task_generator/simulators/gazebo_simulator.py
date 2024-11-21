import rclpy

from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity, SetEntityPose, ControlWorld
from ros_gz_interfaces.msg import EntityFactory, WorldControl
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
import traceback
from task_generator.utils.geometry import quaternion_from_euler
from task_generator.simulators import BaseSimulator

from task_generator.shared import ModelType, Namespace, PositionOrientation, RobotProps


class GazeboSimulator(BaseSimulator):
    def __init__(self, namespace):
        """Initialize GazeboSimulator

        Args:
            namespace: Namespace for the simulator
        """
        # Ensure we have a valid node name
        node_name = namespace.strip('/').replace('/', '_')
        if not node_name:
            node_name = "gazebo_simulator"  # Default name if namespace is empty

        super().__init__(namespace=Namespace(node_name))

        self.node.get_logger().info(
            f"Initializing GazeboSimulator with namespace: {namespace}")
        self._goal_pub = self.node.create_publisher(
            PoseStamped,
            self._namespace("/goal"),
            10
        )

        # Robot model parameter
        self.node.declare_parameter('robot_model', '')
        self._robot_name = self.node.get_parameter('robot_model').value
        self.node.get_logger().info(f"Using robot model: {self._robot_name}")

        # Initialize service clients
        try:

            self.node.get_logger().info("Creating Gazebo service clients...")

            self._spawn_entity = self.node.create_client(
                SpawnEntity, '/world/default/create')
            self._delete_entity = self.node.create_client(
                DeleteEntity, '/world/default/remove')
            self._set_entity_pose = self.node.create_client(
                SetEntityPose, '/world/default/set_pose')
            self._control_world = self.node.create_client(
                ControlWorld, '/world/default/control')

            self.node.get_logger().info("Successfully created all service clients")

        except Exception as e:
            self.node.get_logger().error(
                f"Error creating service clients: {str(e)}")
            traceback.print_exc()
            return

        self.node.get_logger().info("Waiting for Gazebo services...")
        services = [
            (self._spawn_entity, "Spawn entity"),
            (self._delete_entity, "Delete entity"),
            (self._set_entity_pose, "Set entity pose"),
            (self._control_world, "Control world")
        ]

        services_ready = True
        for service, name in services:
            if not self._wait_for_service(service, name):
                services_ready = False

        if not services_ready:
            self.node.get_logger().error("Not all Gazebo services are available!")
            return

        self.node.get_logger().info("All Gazebo services are available now.")
        # resp_spawn = self._spawn_entity.call_async(req)
        # resp_delete = self._delete_entity.call_async(req)
        # resp_pose = self._set_entity_pose.call_async(req)
        # resp_world = self._control_world.call_async(req)
        # rclpy.spin_until_future_complete(this._node, resp_spawn)
        # rclpy.spin_until_future_complete(this._node, resp_delete)
        # rclpy.spin_until_future_complete(this._node, resp_pose)
        # rclpy.spin_until_future_complete(this._node, resp_world)

    def before_reset_task(self):
        self.node.get_logger().info("Pausing simulation before reset")
        self.pause_simulation()

    def after_reset_task(self):
        self.node.get_logger().info("Unpausing simulation after reset")
        try:
            self.unpause_simulation()
        except Exception as e:
            self.node.get_logger().error(
                f"Error unpausing simulation: {str(e)}")
            traceback.print_exc()
            raise

    def _wait_for_service(self, service, name, timeout=15.0):
        self.node.get_logger().info(
            f"Waiting for {name} service (timeout: {timeout}s)...")
        timeout_loop = timeout
        while not service.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().warn(
                f"{name} service not available, waiting... ({timeout_loop}s remaining)")
            timeout_loop -= 1.0
            if timeout_loop <= 0:
                self.node.get_logger().error(
                    f"{name} service not available after {timeout} seconds")
                return False
        self.node.get_logger().info(f"{name} service is now available")
        return True

    def move_entity(self, name, position):
        self.node.get_logger().info(
            f"Moving entity {name} to position: {position}")
        request = SetEntityPose.Request()
        request.entity = name
        request.pose = Pose(
            position=Point(x=position.x, y=position.y, z=0),
            orientation=Quaternion(
                *quaternion_from_euler(0.0, 0.0, position.orientation, axes="sxyz"))
        )

        try:
            future = self._set_entity_pose.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()

            if result is None:
                self.node.get_logger().error(
                    f"Move service call failed for {name}")
                return False

            self.node.get_logger().info(
                f"Move result for {name}: {
                    result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(
                f"Error moving entity {name}: {str(e)}")
            traceback.print_exc()
            return False

    def spawn_entity(self, entity):
        self.node.get_logger().info(
            f"Attempting to spawn entity: {
                entity.name}")

        # Check service availability
        if not self._spawn_entity.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error("Spawn service not available!")
            return False

        try:
            # Create spawn request
            request = SpawnEntity.Request()
            request.entity_factory = EntityFactory()
            request.entity_factory.name = entity.name
            request.entity_factory.type = 'sdf'

            # Get model description
            model_description = entity.model.get(self.MODEL_TYPES).description
            request.entity_factory.sdf = model_description

            self.node.get_logger().info(
                f"Model description length for {
                    entity.name}: {
                    len(model_description)}")

            # Set pose
            request.entity_factory.pose = Pose(
                position=Point(x=entity.position.x, y=entity.position.y, z=0),
                orientation=Quaternion(
                    *
                    quaternion_from_euler(
                        0.0,
                        0.0,
                        entity.position.orientation,
                        axes="sxyz"))
            )

            self.node.get_logger().info(
                f"Spawn position for {
                    entity.name}: x={
                    entity.position.x}, y={
                    entity.position.y}")

            # For robots, declare additional parameters
            if isinstance(entity, RobotProps):
                self.node.get_logger().info(
                    f"Entity {
                        entity.name} is a robot, declaring additional parameters")
                self.node.declare_parameter(
                    Namespace(entity.name)("robot_description"),
                    entity.model.get(self.MODEL_TYPES).description
                )
                self.node.declare_parameter(
                    Namespace(entity.name)("tf_prefix"),
                    str(Namespace(entity.name))
                )

            # Send spawn request
            self.node.get_logger().info(
                f"Sending spawn request for {
                    entity.name}")
            future = self._spawn_entity.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()

            if result is None:
                self.node.get_logger().error(
                    f"Spawn service call failed for {
                        entity.name}")
                return False

            self.node.get_logger().info(
                f"Spawn result for {
                    entity.name}: {
                    result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(
                f"Error spawning entity {
                    entity.name}: {
                    str(e)}")
            traceback.print_exc()
            return False

    def delete_entity(self, name: str):
        self.node.get_logger().info(f"Attempting to delete entity: {name}")
        request = DeleteEntity.Request()
        request.entity = name

        try:
            future = self._delete_entity.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()

            if result is None:
                self.node.get_logger().error(
                    f"Delete service call failed for {name}")
                return False

            self.node.get_logger().info(
                f"Delete result for {name}: {
                    result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(
                f"Error deleting entity {name}: {str(e)}")
            traceback.print_exc()
            return False

    def pause_simulation(self):
        self.node.get_logger().info("Attempting to pause simulation")
        request = ControlWorld.Request()
        request.world_control = WorldControl()
        request.world_control.pause = True

        try:
            future = self._control_world.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()

            if result is None:
                self.node.get_logger().error("Pause service call failed")
                return False

            self.node.get_logger().info(f"Pause result: {result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(f"Error pausing simulation: {str(e)}")
            traceback.print_exc()
            return False

    def unpause_simulation(self):
        self.node.get_logger().info("Attempting to unpause simulation")
        request = ControlWorld.Request()
        request.world_control = WorldControl()
        request.world_control.pause = False

        try:
            future = self._control_world.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()

            if result is None:
                self.node.get_logger().error("Unpause service call failed")
                return False

            self.node.get_logger().info(f"Unpause result: {result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(
                f"Error unpausing simulation: {str(e)}")
            traceback.print_exc()
            return False

    def step_simulation(self, steps):
        self.node.get_logger().info(f"Stepping simulation by {steps} steps")
        request = ControlWorld.Request()
        request.world_control = WorldControl()
        request.world_control.multi_step = steps

        try:
            future = self._control_world.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()

            if result is None:
                self.node.get_logger().error("Step service call failed")
                return False

            self.node.get_logger().info(f"Step result: {result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(
                f"Error stepping simulation: {str(e)}")
            traceback.print_exc()
            return False

    def _publish_goal(self, goal: PositionOrientation):
        self.node.get_logger().info(
            f"Publishing goal: x={
                goal.x}, y={
                goal.y}, orientation={
                goal.orientation}")
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal.x
        goal_msg.pose.position.y = goal.y
        goal_msg.pose.orientation = Quaternion(
            *quaternion_from_euler(0.0, 0.0, goal.orientation, axes="sxyz"))

        self._goal_pub.publish(goal_msg)
        self.node.get_logger().info("Goal published")
