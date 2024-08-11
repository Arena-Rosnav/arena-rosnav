import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, DeleteModel, SetModelState
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from std_srvs.srv import Empty

from task_generator.simulators import SimulatorFactory
from task_generator.shared import rosparam_get
from tf_transformations import quaternion_from_euler
from task_generator.constants import Config, Constants
from task_generator.simulators import BaseSimulator

from task_generator.shared import ModelType, Namespace, PositionOrientation, RobotProps

class GazeboSimulator(BaseSimulator, Node):

    def __init__(self, namespace):
        # Ensure we have a valid node name
        node_name = namespace.strip('/').replace('/', '_')
        if not node_name:
            node_name = "gazebo_simulator"  # Default name if namespace is empty
        
        Node.__init__(self, node_name)
        super().__init__(namespace)
        
        self._goal_pub = self.create_publisher(
            PoseStamped,
            self._namespace("/goal"),
            10
        )
        self.declare_parameter('robot_model', '')
        self._robot_name = self.get_parameter('robot_model').value

        # Initialize service clients
        self._spawn_model = {
            ModelType.URDF: self.create_client(SpawnModel, '/gazebo/spawn_urdf_model'),
            ModelType.SDF: self.create_client(SpawnModel, '/gazebo/spawn_sdf_model'),
        }
        
        self._move_model_srv = self.create_client(SetModelState, '/gazebo/set_model_state')
        self._unpause = self.create_client(Empty, '/gazebo/unpause_physics')
        self._pause = self.create_client(Empty, '/gazebo/pause_physics')
        self._remove_model_srv = self.create_client(DeleteModel, '/gazebo/delete_model')
        
        self.get_logger().info("Waiting for gazebo services...")

        # Wait for services to be available
        self._unpause.wait_for_service()
        self._pause.wait_for_service()
        self._remove_model_srv.wait_for_service()
        self._spawn_model[ModelType.SDF].wait_for_service()
        self._move_model_srv.wait_for_service()

        self.get_logger().info("Gazebo services are available now.")


    def before_reset_task(self):
        self._pause()

    def after_reset_task(self):
        try:
            self._unpause()
        except rospy.service.ServiceException as e:  # gazebo isn't the most reliable
            rospy.logwarn(e)

    # ROBOT

    def move_entity(self, name, position):
        request = SetModelState.Request()
        request.model_state = ModelState()
        request.model_state.model_name = name
        pose = Pose()
        pose.position.x = position.x
        pose.position.y = position.y
        pose.position.z = 0
        pose.orientation = Quaternion(
            *quaternion_from_euler(0.0, 0.0, position.orientation, axes="sxyz")
        )
        request.model_state.pose = pose
        request.model_state.reference_frame = "world"

        future = self._move_model_srv.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def spawn_entity(self, entity):
        request = SpawnModel.Request()

        model = entity.model.get(self.MODEL_TYPES)

        request.model_name = entity.name
        request.model_xml = model.description
        request.initial_pose = Pose(
            position=Point(
                x=entity.position.x,
                y=entity.position.y,
                z=0
            ),
            orientation=Quaternion(
                *quaternion_from_euler(0.0, 0.0, entity.position.orientation, axes="sxyz")
            )
        )
        request.robot_namespace = Namespace(entity.name)
        request.reference_frame = "world"

        if isinstance(entity, RobotProps):
            self.declare_parameter(request.robot_namespace("robot_description"), model.description)
            self.declare_parameter(request.robot_namespace("tf_prefix"), str(request.robot_namespace))

        future = self._spawn_model[model.type].call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success


    def delete_entity(self, name):
        request = DeleteModel.Request()
        request.model_name = name
        future = self._remove_model_srv.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def _publish_goal(self, goal: PositionOrientation):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal.x
        goal_msg.pose.position.y = goal.y
        goal_msg.pose.orientation = Quaternion(
            *quaternion_from_euler(0.0, 0.0, goal.orientation, axes="sxyz"))

        self._goal_pub.publish(goal_msg)
