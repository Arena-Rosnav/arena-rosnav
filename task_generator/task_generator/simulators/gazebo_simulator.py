import dataclasses
import math
import typing

import rclpy
import numpy as np

from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity, SetEntityPose, ControlWorld
from ros_gz_interfaces.msg import EntityFactory, WorldControl
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
import traceback
from task_generator.utils.geometry import quaternion_from_euler
from task_generator.simulators import BaseSimulator

from task_generator.shared import ModelType, Namespace, PositionOrientation, RobotProps, Robot, Wall

import launch
import launch_ros
import ament_index_python


class GazeboSimulator(BaseSimulator):

    _walls_entities: typing.List[str]

    def __init__(self, namespace):
        """Initialize GazeboSimulator

        Args:
            namespace: Namespace for the simulator
        """

        super().__init__(namespace=namespace)

        self.node.get_logger().info(
            f"Initializing GazeboSimulator with namespace: {namespace}")
        self._goal_pub = self.node.create_publisher(
            PoseStamped,
            self._namespace("goal"),
            10
        )
        self.entities = {}
        self._walls_entities = []

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
        return True
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
            f"Attempting to move entity: {name}")
        entity = self.entities[name]
        self.delete_entity(name)
        entity = dataclasses.replace(entity, position=position)
        self.spawn_entity(entity)
        return True
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
                f"Move result for {name}: {result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(
                f"Error moving entity {name}: {str(e)}")
            traceback.print_exc()
            return False

    def spawn_entity(self, entity):
        self.node.get_logger().info(
            f"Attempting to spawn entity: {entity.name}")

        launch_description = launch.LaunchDescription()

        try:
            description = entity.model.get(
                [ModelType.SDF, ModelType.URDF]).description
        except FileNotFoundError as e:
            self.node.get_logger().error(repr(e))
            return True

        launch_description.add_action(
            launch_ros.actions.Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    '-world', 'default',
                    '-string', description,
                    '-name', entity.name,
                    '-allow_renaming', 'false',
                    '-x', str(entity.position.x),
                    '-y', str(entity.position.y),
                    '-Y', str(entity.position.orientation)  # Corrected from '-X' to '-Y'
                ],
            )
        )

        gz_topic = '/model/' + entity.name

        # Bridge to connect Gazebo and ROS2
        if isinstance(entity, RobotProps):
            launch_description.add_action(
                launch_ros.actions.Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    output='screen',
                    arguments=[
                        # Odometry (Gazebo -> ROS2)
                        gz_topic + '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        # IMU (Gazebo -> ROS2)
                        '/world/default/model/' + entity.name + '/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                        # Velocity command (ROS2 -> Gazebo)
                        gz_topic + '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        # LiDAR Scan (Gazebo -> ROS2)
                        '/world/default/model/' + entity.name + '/link/base_link/sensor/gpu_lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                        # LiDAR Point Cloud (Gazebo -> ROS2)
                        '/world/default/model/' + entity.name + '/link/base_link/sensor/gpu_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                        # Sensors Marker (if needed, Gazebo -> ROS2)
                        '/sensors/marker@visualization_msgs/msg/Marker[gz.msgs.Visual',
                        # TF Data (Gazebo -> ROS2)
                        gz_topic + '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        gz_topic + '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
                    ],
                    remappings=[
                        # Remap Gazebo topics to ROS2 topics
                        (gz_topic + '/tf', '/tf'),
                        (gz_topic + '/odometry', entity.name + '/odom'),
                        ('/world/default/model/' + entity.name + '/link/base_link/sensor/imu_sensor/imu', entity.name + '/imu/data'),
                        (gz_topic + '/cmd_vel', entity.name + '/cmd_vel'),
                        ('/world/default/model/' + entity.name + '/link/base_link/sensor/gpu_lidar/scan', entity.name + '/lidar'),
                        ('/world/default/model/' + entity.name + '/link/base_link/sensor/gpu_lidar/scan/points', entity.name + '/lidar/points'),
                        ('/sensors/marker', entity.name + '/marker')
                    ],
                    parameters=[
                        {
                            'use_sim_time': True
                        }
                    ],
                )
            )
            launch_description.add_action(
                launch_ros.actions.Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    output='screen',
                    parameters=[
                        {'use_sim_time': True},
                        {'robot_description': description},
                        {'frame_prefix': entity.frame}
                    ],
                    remappings=[
                        ('/tf', 'tf'),
                        ('/tf_static', 'tf_static')
                    ]
                )
            )

            launch_description.add_action(
                launch_ros.actions.Node(
                    package='joint_state_publisher',
                    executable='joint_state_publisher',
                    output='screen',
                    parameters=[
                        {'use_sim_time': True}
                    ]
                )
            )
        self.entities[entity.name] = entity
        self.node.do_launch(launch_description)
        return True
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
                f"Model description length for {entity.name}: { len(model_description)}")

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
                f"Spawn position for {entity.name}: x={entity.position.x}, y={entity.position.y}")

            # For robots, declare additional parameters
            if isinstance(entity, RobotProps):
                self.node.get_logger().info(
                    f"Entity {entity.name} is a robot, declaring additional parameters")
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
                f"Sending spawn request for {entity.name}")
            future = self._spawn_entity.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            result = future.result()

            if result is None:
                self.node.get_logger().error(
                    f"Spawn service call failed for {entity.name}")
                return False

            self.node.get_logger().info(
                f"Spawn result for {entity.name}: {result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(
                f"Error spawning entity {entity.name}: {str(e)}")
            traceback.print_exc()
            return False

    def delete_entity(self, name: str):
        self.node.get_logger().info(
            f"Attempting to delete entity: {name}")

        if name in self.entities:
            del self.entities[name]

        launch_description = launch.LaunchDescription()

        launch_description.add_action(
            launch_ros.actions.Node(
                package="ros_gz_sim",
                executable="remove",
                output="screen",
                parameters=[
                    {
                        "world": "default",
                        "entity_name": name,
                    }
                ],
            )
        )

        self.node.do_launch(launch_description)
        return True
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
                f"Delete result for {name}: {result.success}")
            return result.success

        except Exception as e:
            self.node.get_logger().error(
                f"Error deleting entity {name}: {str(e)}")
            traceback.print_exc()
            return False

    def pause_simulation(self):
        return True
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
        return True
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
        return True
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
            f"Publishing goal: x={goal.x}, y={goal.y}, orientation={goal.orientation}")
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = goal.x
        goal_msg.pose.position.y = goal.y
        goal_msg.pose.orientation = Quaternion(
            *quaternion_from_euler(0.0, 0.0, goal.orientation, axes="sxyz"))

        self._goal_pub.publish(goal_msg)
        self.node.get_logger().info("Goal published")

    def spawn_walls(self, walls) -> bool:
        wall_name = f"custom_wall_{len(self._walls_entities)}"
        # wall_positions = [(0, 0), (5, 0), (5, 5), (0, 5), (0, 0)]  # A square wall
        wall_height = 3.0  # Wall height in meters
        wall_thickness = 0.2  # Wall thickness in meters
        base_position = (0, 0, 0)  # Offset the wall to (10, 10, 0)

        self.node.get_logger().info(f"Attempting to spawn walls: {wall_name}")

        launch_description = launch.LaunchDescription()

        # Generate the SDF string for walls
        wall_sdf = self.generate_wall_sdf(
            name=wall_name,
            walls=walls,
            height=wall_height,
            thickness=wall_thickness,
            base_position=base_position
        )

        if not wall_sdf:
            self.node.get_logger().error(f"Failed to generate SDF for walls: {wall_name}")
            return False

        # Add the wall spawning node
        launch_description.add_action(
            launch_ros.actions.Node(
                package="ros_gz_sim",
                executable="create",
                output="screen",
                arguments=[
                    '-world', 'default',
                    '-string', wall_sdf,
                    '-name', wall_name,
                    '-allow_renaming', 'false',
                ],
            )
        )

        # Store the wall entity and launch
        self._walls_entities.append(wall_name)
        self.node.do_launch(launch_description)

        return True

    def remove_walls(self) -> bool:
        for entity in self._walls_entities:
            self.delete_entity(entity)
        self._walls_entities = []
        return True

    def generate_wall_sdf(
        self,
        name: str,
        walls: typing.List[Wall],
        height: float,
        thickness: float,
        base_position: typing.Tuple[float, float, float] = (0, 0, 0),
    ) -> str:
        """
        Generate an SDF string for a wall structure based on given parameters and base position.
        """
        try:
            sdf_template = """
            <sdf version="1.6">
                <model name="{name}">
                    <pose>{base_x} {base_y} {base_z} 0 0 0</pose>
                    {links}
                    <static>true</static>
                </model>
            </sdf>
            """
            link_template = """
            <link name="wall_segment_{index}">
                <visual name="visual">
                    <geometry>
                        <box>
                            <size>{length} {thickness} {height}</size>
                        </box>
                    </geometry>
                    <material>
                        <ambient>0.7 0.7 0.7 1</ambient>
                    </material>
                </visual>
                <collision name="collision">
                    <geometry>
                        <box>
                            <size>{length} {thickness} {height}</size>
                        </box>
                    </geometry>
                </collision>
                <pose>{x} {y} {z} 0 0 {orientation}</pose>
            </link>
            """
            links = []
            base_x, base_y, base_z = base_position
            z = height / 2.0  # Center the wall height relative to the base

            for i, w in enumerate(walls):
                x1, y1, x2, y2 = w.Start.x, w.Start.y, w.End.x, w.End.y
                length = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
                orientation = math.atan2(y2 - y1, x2 - x1)
                x = (x1 + x2) / 2 + base_x
                y = (y1 + y2) / 2 + base_y

                links.append(
                    link_template.format(
                        index=i,
                        length=length,
                        thickness=thickness,
                        height=height,
                        x=x,
                        y=y,
                        z=z + base_z,
                        orientation=orientation
                    )
                )

            return sdf_template.format(
                name=name,
                base_x=base_x,
                base_y=base_y,
                base_z=base_z,
                links="\n".join(links)
            )

        except Exception as e:
            self.node.get_logger().error(f"Error generating SDF: {repr(e)}")
            return None
