#!/usr/bin/env python3

import rclpy
import os
import traceback
import yaml
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Empty 
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node


class VisualizeRobotModel(Node):
    def __init__(self):
        super().__init__('visualize_robot_model')
        
        self.srv_start_setup = self.create_service(
            Empty,
            "start_model_visualization",
            self.start_model_visualization_callback
        )

        self.robot_models = {}
        self.publisher_map = {}
        self.subscribers = []

    def start_model_visualization_callback(self, request, response):
        # Declare parameters for robot_names
        self.declare_parameter('robot_names', [])
        robot_names = self.get_parameter('robot_names').value

        # Get the appropriate odom topic based on complexity
        robot_odom_topic = self.get_complexity_odom_topic()

        for name in robot_names:
            # Get robot model name from parameter or use a default
            self.declare_parameter(f'{name}.robot_model', 'jackal')
            robot_model = self.get_parameter(f'{name}.robot_model').value

            # Load the model file
            model_file = self.read_robot_model_file(robot_model)

            # Generate markers for the model
            markers_for_model = self.create_marker_array_for_robot(model_file)

            # Store the markers
            self.robot_models[robot_model] = markers_for_model

            # Create publisher for each robot
            self.publisher_map[name] = self.create_publisher(
                MarkerArray,
                os.path.join(name, "visualize", "model"),
                10
            )

            # Create subscriber for each robot's odometry
            self.subscribers.append(
                self.create_subscription(
                    Odometry,
                    os.path.join(name, robot_odom_topic),
                    lambda msg, args=(robot_model, name): self.publish_model(msg, args),
                    10
                )
            )

        return Empty.Response()

    def publish_model(self, data, args):
        robot_model, name = args

        try:
            markers = self.robot_models[robot_model]
        except Exception:
            self.get_logger().error(f"Error - Getting markers from dict {robot_model}")
            return

        for marker in markers:
            marker.header = data.header
            marker.header.frame_id = "map"
            marker.pose = data.pose.pose

        try:
            self.publisher_map[name].publish(MarkerArray(markers=markers))
        except Exception:
            self.get_logger().error(traceback.format_exc())
            self.get_logger().error(f"Error - publishing markers {name}")

    def read_robot_model_file(self, robot_model):
        try:
            # In ROS2, use ament_index_python instead of rospkg
            from ament_index_python.packages import get_package_share_directory
            
            file_path = os.path.join(
                get_package_share_directory('simulation_setup'),
                "entities",
                "robots",
                robot_model,
                f"{robot_model}.model.yaml"
            )

            with open(file_path) as file:
                return yaml.safe_load(file)["bodies"]
        except Exception as e:
            self.get_logger().error(f"Error reading robot model file: {e}")
            return self.get_default_model()

    def get_default_model(self):
        # Simple default model if the real one can't be found
        return [{
            'color': [0, 0, 1, 1],
            'footprints': [
                {
                    'type': 'circle',
                    'radius': 0.25
                }
            ]
        }]

    def create_marker_array_for_robot(self, bodies):
        markers = []

        for i, body in enumerate(bodies):
            color = body.get("color", [0, 0, 1, 1])

            for j, footprint in enumerate(body.get("footprints", [])):
                marker = self.create_marker_from_footprint(footprint, color, i * 100 + j)
                markers.append(marker)

        return markers

    def create_marker_from_footprint(self, footprint, color, id):
        r, g, b, a = color

        marker = Marker()
        marker.id = id
        marker.action = Marker.ADD
        marker.color = ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))

        if footprint.get("type") == "circle":
            marker.type = Marker.SPHERE
            marker.scale = Vector3(
                x=footprint.get("radius", 0.25) * 2,
                y=footprint.get("radius", 0.25) * 2,
                z=0.1
            )
        else:
            marker.type = Marker.LINE_STRIP
            marker.scale = Vector3(x=0.03, y=0, z=0)

            for x, y in footprint.get("points", []):
                marker.points.append(Point(x=float(x), y=float(y), z=0.0))

            # Close the loop by adding the first point again
            if marker.points and len(marker.points) > 0:
                marker.points.append(marker.points[0])

        return marker

    def get_complexity_odom_topic(self):
        self.declare_parameter('complexity', 1)
        complexity = self.get_parameter('complexity').value

        if complexity == 1:
            return "odom"
        elif complexity == 2:
            return "odom_amcl"
        else:
            return "odom"  # Default


def main(args=None):
    rclpy.init(args=args)
    
    visualizer = VisualizeRobotModel()
    
    rclpy.spin(visualizer)
    
    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()