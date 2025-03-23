#! /usr/bin/env python3

import os
import sys
import tempfile
import time
import typing

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

    topics: typing.List[typing.Tuple[str, typing.List[str]]]
    robot_names: typing.List[str]

    def _wait_for_param(
        self,
        client: rclpy.client.Client,
        param_name: str,
        test_fn: typing.Callable[[typing.Any], bool] | None = None,
    ) -> rcl_interfaces.msg.ParameterValue:
        """
        Block execution until parameter passes test function.
        @parameter client: rclpy GetParameters service client
        @paramter parameter_name: name of parameter
        @test_fn: test function for parameter
        """
        while True:
            req = rcl_interfaces.srv.GetParameters.Request(names=[param_name])
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            params = future.result()
            if params.values:
                value = params.values[0]
                if (not test_fn) or test_fn(value):
                    self.get_logger().info(f'param {param_name} is set')
                    return value
            self.get_logger().info(f'waiting for {param_name} to be set')
            time.sleep(1)

    def __init__(self):
        Node.__init__(self, 'create_rviz_config_file')

        TASKGEN_NODE = '/task_generator_node'
        TASKGEN_PARAM_SRV = os.path.join(TASKGEN_NODE, 'get_parameters')
        PARAM_INITIALIZED = 'initialized'

        get_parameters_cli = self.create_client(rcl_interfaces.srv.GetParameters, TASKGEN_PARAM_SRV)
        while not get_parameters_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'waiting for service {TASKGEN_PARAM_SRV} to become available')
        self.get_logger().info(f'service {TASKGEN_PARAM_SRV} is available')

        self._wait_for_param(get_parameters_cli, PARAM_INITIALIZED, lambda x: x.bool_value)
        self.robot_names = self._wait_for_param(get_parameters_cli, 'robot_names').string_array_value

        # self.cli_load = self.create_client('/rviz2/load_config', rcl_interfaces.srv.SetString)

    def create_config(self) -> str:
        default_file = ConfigFileGenerator._read_default_file()

        # cache
        self.topics = self.get_topic_names_and_types()

        displays = []

        # Add the map display
        displays.append({
            'Class': 'rviz_default_plugins/Map',
            'Enabled': True,
            'Name': 'Map',
            'Topic': {
                'Value': '/map',
                'Depth': 5,
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
        default_file["Visualization Manager"]["Views"]["Current"] = {
            "Class": "rviz_default_plugins/Orbit",
            "Distance": 10.0,
            "Focal Point": {
                "X": 0.0,
                "Y": 0.0,
                "Z": 0.0
            },
            "Name": "Current View",
            "Near Clip Distance": 0.01,
            "Pitch": 0.5,
            "Target Frame": "<Fixed Frame>",
            "Value": True,
            "Yaw": 0.0
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
        robot_model_display = {
            'Class': 'rviz_default_plugins/RobotModel',
            'Name': 'Robot Model',
            'Enabled': True,
            'TF Prefix': robot_name,
            'Description Topic': {
                'Value': f'/task_generator_node/{robot_name}/robot_description',
                'Depth': 5,
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Reliable',
                'Durability Policy': 'Volatile',
            },
            'Visual Enabled': True,
            'Collision Enabled': False
        }
        robot_group['Displays'].append(robot_model_display)

        # Add odometry visualization
        odom_topic = f'/task_generator_node/{robot_name}/odom'
        odom_display = {
            'Class': 'rviz_default_plugins/Odometry',
            'Name': 'Odometry',
            'Enabled': True,
            'Topic': {
                'Value': odom_topic,
                'Depth': 5,
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Reliable',
                'Durability Policy': 'Volatile',
            },
            'Shape': 'Arrow',
            'Color': color,
            'Position Tolerance': 0.1,
            'Angle Tolerance': 0.1,
            'Keep': 1,
            'Shaft Length': 0.5,
            'Shaft Radius': 0.05,
            'Head Length': 0.2,
            'Head Radius': 0.1
        }
        robot_group['Displays'].append(odom_display)

        # Add local costmap
        local_costmap_topic = f'/task_generator_node/{robot_name}/local_costmap/costmap'
        local_costmap_display = {
            'Class': 'rviz_default_plugins/Map',
            'Name': 'Local Costmap',
            'Enabled': True,
            'Topic': {
                'Value': local_costmap_topic,
                'Depth': 5,
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Reliable',
                'Durability Policy': 'Transient Local',
            },
            'Color Scheme': 'costmap',
            'Draw Behind': False,
            'Alpha': 0.7
        }
        robot_group['Displays'].append(local_costmap_display)

        # Add global costmap
        global_costmap_topic = f'/task_generator_node/{robot_name}/global_costmap/costmap'
        global_costmap_display = {
            'Class': 'rviz_default_plugins/Map',
            'Name': 'Global Costmap',
            'Enabled': True,
            'Topic': {
                'Value': global_costmap_topic,
                'Depth': 5,
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Reliable',
                'Durability Policy': 'Transient Local',
            },
            'Color Scheme': 'costmap',
            'Draw Behind': False,
            'Alpha': 0.7
        }
        robot_group['Displays'].append(global_costmap_display)

        # Add path visualization
        path_topic = f'/task_generator_node/{robot_name}/plan'
        path_display = {
            'Class': 'rviz_default_plugins/Path',
            'Name': 'Global Plan',
            'Enabled': True,
            'Topic': {
                'Value': path_topic,
                'Depth': 5,
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Reliable',
                'Durability Policy': 'Volatile',
            },
            'Color': color,
            'Line Width': 0.05
        }
        robot_group['Displays'].append(path_display)

        # Add local path visualization
        local_path_topic = f'/task_generator_node/{robot_name}/local_plan'
        local_path_display = {
            'Class': 'rviz_default_plugins/Path',
            'Name': 'Local Plan',
            'Enabled': True,
            'Topic': {
                'Value': local_path_topic,
                'Depth': 5,
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Reliable',
                'Durability Policy': 'Volatile',
            },
            'Color': '255; 0; 0',  # Red for local path
            'Line Width': 0.05
        }
        robot_group['Displays'].append(local_path_display)

        # Add robot footprint
        footprint_topic = f'/task_generator_node/{robot_name}/local_costmap/published_footprint'
        footprint_display = {
            'Class': 'rviz_default_plugins/Polygon',
            'Name': 'Robot Footprint',
            'Enabled': True,
            'Topic': {
                'Value': footprint_topic,
                'Depth': 5,
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Reliable',
                'Durability Policy': 'Volatile',
            },
            'Color': color,
            'Alpha': 1.0
        }
        robot_group['Displays'].append(footprint_display)

        # SENSORS

        # TODO move this to the class
        # Add laser scan visualization
        def laser_display(laser_topic):
            return {
                'Class': 'rviz_default_plugins/LaserScan',
                'Name': 'LaserScan',
                'Enabled': True,
                'Topic': {
                    'Value': laser_topic,
                    'Depth': 5,
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Best Effort',
                    'Durability Policy': 'Volatile',
                },
                'Color': color,
                'Size (m)': 0.05,
                'Style': 'Points',
                'Alpha': 1.0,
                'Decay Time': 0.0
            }

        sensor_displays = {
            'sensor_msgs/msg/LaserScan': laser_display
        }

        robot_ns = os.path.join('/task_generator_node', robot_name)
        robot_topics = (t for t in self.topics if t[0].startswith(robot_ns))

        for topic_name, topic_types in robot_topics:
            matched_display = next((sensor_displays[t] for t in topic_types if t in sensor_displays), None)
            if matched_display is not None:
                robot_group['Displays'].append(matched_display(topic_name))

        return robot_group

    # def _create_display_for_topic(self, robot_name, topic, color):
    #     matchers = [
    #         (Matcher.GLOBAL_PLAN, Config.create_path_display),
    #         (Matcher.LASER_SCAN, Config.create_laser_scan_display),
    #         (Matcher.GLOBAL_COSTMAP, Config.create_global_map_display),
    #         (Matcher.LOCAL_COSTMAP, Config.create_local_map_display),
    #         (Matcher.GOAL, Config.create_pose_display),
    #         (Matcher.MODEL, Config.create_model_display)
    #     ]

    #     for matcher, function in matchers:
    #         match = re.search(matcher(robot_name), topic)

    #         if match:
    #             return function(robot_name, topic, color)

    # def _send_load_config(self, file_path):
    #     # print(f"Attempting to call /rviz/load_config with file: {file_path}")
    #     while not self.cli_load.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('waiting for service /rviz/load_config to become available')
    #     self.cli_load.call(file_path)
    #     # print("Call to /rviz/load_config completed.")

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

    config_file_generator = ConfigFileGenerator()
    try:
        config_file = config_file_generator.create_config()
        launch_service = launch.launch_service.LaunchService()
        launch_service.include_launch_description(
            launch.LaunchDescription([
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
        rclpy.shutdown()

    sys.exit(0)


if __name__ == "__main__":
    main()
