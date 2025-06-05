#! /usr/bin/env python3

import os
import sys
import tempfile
import time
import typing

import arena_bringup.extensions.NodeLogLevelExtension as NodeLogLevelExtension
import launch
import launch.launch_service
import launch_ros.actions
import rcl_interfaces.msg
import rcl_interfaces.srv
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rviz_utils.utils import Utils


class ConfigFileGenerator(Node):

    _origin: typing.List[float]
    topics: typing.List[typing.Tuple[str, typing.List[str]]]
    robot_names: typing.List[str]

    def _wait_for_param(
        self,
        client: rclpy.client.Client,
        param_name: str,
        test_fn: typing.Callable[[typing.Any], bool] | None = None,
        timeout: float = 1,
    ) -> rcl_interfaces.msg.ParameterValue:
        """
        Block execution until parameter passes test function.
        @parameter client: rclpy GetParameters service client
        @paramter parameter_name: name of parameter
        @test_fn: test function for parameter
        @timeout: timeout in seconds
        """
        while True:
            self.get_logger().info(f'waiting for {param_name} to be set')
            for _ in range(5):
                req = rcl_interfaces.srv.GetParameters.Request(names=[param_name])
                future = client.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
                params = future.result()
                if params and params.values:
                    value = params.values[0]
                    if (not test_fn) or test_fn(value):
                        self.get_logger().info(f'param {param_name} is set')
                        return value
                time.sleep(timeout)

    def __init__(self, TASKGEN_NODE: str = '/task_generator_node'):
        Node.__init__(self, 'rviz_config_generator')

        self._TASKGEN_NODE = TASKGEN_NODE

        self.declare_parameter('origin', [0.0, 0.0, 0.0])
        origin = self.get_parameter('origin').value
        self._origin = list((*origin, 0.0, 0.0, 0.0)[:3])

        TASKGEN_PARAM_SRV = os.path.join(self._TASKGEN_NODE, 'get_parameters')
        PARAM_INITIALIZED = 'initialized'

        get_parameters_cli = self.create_client(rcl_interfaces.srv.GetParameters, TASKGEN_PARAM_SRV)
        while not get_parameters_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'waiting for service {TASKGEN_PARAM_SRV} to become available')
        self.get_logger().info(f'service {TASKGEN_PARAM_SRV} is available')

        self._wait_for_param(get_parameters_cli, PARAM_INITIALIZED, lambda x: x.bool_value)
        self.robot_names = self._wait_for_param(get_parameters_cli, 'robot_names').string_array_value

        # self.cli_load = self.create_client('/rviz2/load_config', rcl_interfaces.srv.SetString)

    def _create_pedestrian_group(self):
            """Creates a Pedestrian Group with stylized human visualizations"""
            
            pedestrian_group = {
                'Class': 'rviz_common/Group',
                'Name': 'Pedestrians',
                'Enabled': True,
                'Displays': []
            }

            # Check if pedestrian topics exist
            pedestrian_topics = []
            for topic_name, topic_types in self.topics:
                if topic_name == '/people' and 'people_msgs/msg/People' in topic_types:
                    pedestrian_topics.append(('/people', 'people_msgs/msg/People'))
                elif topic_name == '/human_states' and 'hunav_msgs/msg/Agents' in topic_types:
                    pedestrian_topics.append(('/human_states', 'hunav_msgs/msg/Agents'))
                elif topic_name == '/pedestrian_markers' and 'visualization_msgs/msg/MarkerArray' in topic_types:
                    pedestrian_topics.append(('/pedestrian_markers', 'visualization_msgs/msg/MarkerArray'))

            if not pedestrian_topics:
                self.get_logger().warn("No pedestrian topics found. Pedestrian group will be empty.")
                return pedestrian_group

            # Add displays for found pedestrian topics
            for topic_name, topic_type in pedestrian_topics:
                if topic_type == 'visualization_msgs/msg/MarkerArray':
                    # Use MarkerArray display for converted pedestrian markers
                    display = Utils.Displays.pedestrians(topic_name)
                    pedestrian_group['Displays'].append(display)
                    self.get_logger().info(f"Added MarkerArray display for pedestrians: {topic_name}")
                    
                elif topic_type == 'people_msgs/msg/People':
                    # Add raw people display as fallback
                    display = Utils.Displays.pedestrians_raw(topic_name)
                    pedestrian_group['Displays'].append(display)
                    self.get_logger().info(f"Added raw People display: {topic_name}")
                    
                elif topic_type == 'hunav_msgs/msg/Agents':
                    # Could add custom agent display here if needed
                    self.get_logger().info(f"Found HuNav agents topic: {topic_name} (not yet implemented)")

            # Add TF display for pedestrian frames (disabled fallback only)
            tf_display = {
                'Class': 'rviz_default_plugins/TF',
                'Name': 'Pedestrian TF Frames',
                'Enabled': False,  # Disabled by default since we have proper markers
                'Frame Timeout': 15,
                'Marker Scale': 0.3,
                'Show Arrows': True,
                'Show Axes': False,
                'Show Names': True,
                # No static tree - frames will be discovered dynamically by RViz
            }
            pedestrian_group['Displays'].append(tf_display)

            return pedestrian_group

    def create_config(self) -> str:
        default_file = self._read_default_file()

        # cache
        self.topics = self.get_topic_names_and_types()

        displays = []

        # Add the map display
        displays.append({
            'Class': 'rviz_default_plugins/Map',
            'Enabled': True,
            'Name': 'Map',
            'Topic': {
                'Value': os.path.join(self._TASKGEN_NODE, 'map'),
                'Depth': 20,
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Reliable',
                'Durability Policy': 'Transient Local',
            },
            'Use Timestamp': False,
            'Alpha': 0.7
        })

        # Add TF display
        displays.append({
            'Class': 'rviz_default_plugins/TF',
            'Enabled': True,
            'Name': 'TF',
            'Frame Timeout': 15,
            'Marker Scale': 1.0,
            'Show Arrows': True,
            'Show Axes': True,
            'Show Names': False
        })

        published_topics = [topic[0] for topic in self.get_topic_names_and_types()]

        for robot_name in self.robot_names:
            robot_group = self._create_robot_group(robot_name)
            displays.append(robot_group)

        # HUNAVSIM: pedestrian group
        pedestrian_group = self._create_pedestrian_group()
        displays.append(pedestrian_group)

        # PedSim configuration - commented out but kept for future use
        # try:
        #     if not self.has_parameter('pedsim'):
        #         self.declare_parameter('pedsim', False)
        #     if self.get_parameter('pedsim').value:
        #         displays.append(Config.TRACKED_PERSONS)
        #         displays.append(Config.TRACKED_GROUPS)
        #         displays.append(Config.PEDSIM_WALLS)
        #         displays.append(Config.PEDSIM_WAYPOINTS)
        # except Exception as e:
        #     self.get_logger().warn(f"Error checking pedsim parameter: {e}")

        # Set the default view to Orbit (instead of TopDownOrtho)

        python_yaw: float = 3.8
        try:
            python_yaw = sum(
                2 * (i % 2 - 0.5) * float(d) / 10**i
                for i, d
                in enumerate(sys.version.split(' ', 1)[0].split('.'))
            )  # i am going insane
        except BaseException:
            pass

        default_file["Visualization Manager"]["Views"]["Current"] = {
            "Class": "rviz_default_plugins/Orbit",
            "Distance": 50.0,
            "Focal Point": {
                "X": 15.0 + self._origin[0],
                "Y": 10.0 + self._origin[1],
                "Z": 0.0 + self._origin[2],
            },
            "Name": "Current View",
            "Near Clip Distance": 0.01,
            "Pitch": 0.9,
            "Target Frame": "<Fixed Frame>",
            "Value": True,
            "Yaw": python_yaw
        }

        default_file["Visualization Manager"]["Displays"] = displays

        file_path = self._tmp_config_file(default_file)
        self.get_logger().info(f'created config file at {file_path}')

        return file_path

    def _start_setup_callback(self, request, response):
        self.get_logger().info("Service callback triggered.")
        file_path = self.create_config()
        self._send_load_config(file_path)
        return response

    def _create_robot_group(self, robot_name):
        """Creates a Robot Group with all visualizations for a robot"""
        color = Utils.get_random_rviz_color()

        robot_group = {
            'Class': 'rviz_common/Group',
            'Name': f'Robot: {robot_name}',
            'Enabled': True,
            'Displays': []
        }

        # Add robot model using RobotModel display
        robot_group['Displays'].append(Utils.Displays.robot_model(robot_name))

        # Add odometry visualization
        odom_topic = f'{self._TASKGEN_NODE}/{robot_name}/odom'
        robot_group['Displays'].append(Utils.Displays.odom(odom_topic, color))

        # Add local costmap
        local_costmap_topic = f'{self._TASKGEN_NODE}/{robot_name}/local_costmap/costmap'
        robot_group['Displays'].append(Utils.Displays.local_costmap(local_costmap_topic))

        # Add global costmap
        global_costmap_topic = f'{self._TASKGEN_NODE}/{robot_name}/global_costmap/costmap'
        robot_group['Displays'].append(Utils.Displays.global_costmap(global_costmap_topic))

        # Add path visualization
        path_topic = f'{self._TASKGEN_NODE}/{robot_name}/plan'
        robot_group['Displays'].append(Utils.Displays.global_path(path_topic, color))

        # Add local path visualization
        local_path_topic = f'{self._TASKGEN_NODE}/{robot_name}/local_plan'
        robot_group['Displays'].append(Utils.Displays.local_path(local_path_topic))

        # Add robot footprint
        footprint_topic = f'{self._TASKGEN_NODE}/{robot_name}/local_costmap/published_footprint'
        robot_group['Displays'].append(Utils.Displays.robot_footprint(footprint_topic, color))

        # SENSORS
        # Map of message types to display creator methods - include all sensor types
        sensor_displays = {
            'sensor_msgs/msg/LaserScan': Utils.Displays.laser_scan,
            'sensor_msgs/msg/PointCloud2': Utils.Displays.pointcloud,
            'sensor_msgs/msg/PointCloud': Utils.Displays.pointcloud_legacy,
            # 'sensor_msgs/msg/Imu': Utils.imu,                          # will be optimised soon
            'foot_contact_msgs/msg/FootContact': Utils.Displays.footcontact,
            'sensor_msgs/msg/Image': Utils.Displays.image,
            # Add more sensor types as needed
        }

        # Track sensor counts for color assignment
        sensor_counts = {}

        # Improved topic discovery for robot sensors
        robot_topics = []

        # Try to discover topics using node-based approach first
        try:
            # Get all nodes in the system
            node_names_and_namespaces = self.get_node_names_and_namespaces()

            # Filter for nodes related to this robot
            robot_nodes = []
            robot_namespace = f'{self._TASKGEN_NODE}/{robot_name}'

            for node_name, node_namespace in node_names_and_namespaces:
                if node_namespace == robot_namespace:
                    robot_nodes.append((node_name, node_namespace))

            self.get_logger().info(f"Found {len(robot_nodes)} nodes for robot {robot_name}")

            # Get topics from each robot node
            for node_name, node_namespace in robot_nodes:
                try:
                    node_topics = self.get_publisher_names_and_types_by_node(node_name, node_namespace)
                    robot_topics.extend(node_topics)
                except Exception as e:
                    self.get_logger().debug(f"Failed to get topics from {node_namespace}/{node_name}: {e}")
        except Exception as e:
            self.get_logger().warning(f"Failed to get topics by node: {e}")

        # Fall back to namespace filtering if node-based discovery failed
        if not robot_topics:
            robot_ns = f'{self._TASKGEN_NODE}/{robot_name}'
            robot_topics = [(t, types) for t, types in self.topics if t.startswith(robot_ns)]
            self.get_logger().info(f"Found {len(robot_topics)} topics using namespace filtering")

        # Add displays for all discovered sensors
        for topic_name, topic_types in robot_topics:
            for topic_type in topic_types:
                if topic_type in sensor_displays:
                    # Track count for this sensor type (for color assignment)
                    if topic_type not in sensor_counts:
                        sensor_counts[topic_type] = 0
                    else:
                        sensor_counts[topic_type] += 1

                    # Get display with appropriate color
                    display_creator = sensor_displays[topic_type]
                    sensor_color = Utils.get_sensor_color(topic_type, sensor_counts[topic_type])
                    display = display_creator(topic_name, sensor_color)

                    robot_group['Displays'].append(display)
                    break  # Use first matching type

        return robot_group

    @staticmethod
    def _read_default_file():
        package_path = get_package_share_directory("rviz_utils")
        file_path = os.path.join(package_path, "config", "rviz_default.rviz")

        with open(file_path) as file:
            return yaml.safe_load(file)

    @classmethod
    def _tmp_config_file(cls, config_file):
        f = tempfile.NamedTemporaryFile('w', delete=False)
        yaml.dump(config_file, f)
        f.close()
        return f.name


def main():
    rclpy.init()

    cli_args = rclpy.utilities.remove_ros_args(sys.argv)
    config_file_generator = ConfigFileGenerator(*cli_args[1:])
    try:
        config_file = config_file_generator.create_config()
        launch_service = launch.launch_service.LaunchService()
        launch_service.include_launch_description(
            launch.LaunchDescription([
                NodeLogLevelExtension.SetGlobalLogLevelAction(
                    rclpy.logging.get_logger_effective_level(config_file_generator.get_logger().name).name.lower()
                ),
                launch_ros.actions.Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    arguments=['-d', config_file],
                    parameters=[{"use_sim_time": True}],
                    output="screen",
                )
            ])
        )
        launch_service.run()
    except KeyboardInterrupt:
        pass
    finally:
        config_file_generator.destroy_node()

    sys.exit(0)


if __name__ == "__main__":
    main()
