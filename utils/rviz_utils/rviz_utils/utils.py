import os
import numpy as np


class Utils:
    @classmethod
    def generate_random_color(cls):
        return list(np.random.choice(range(0, 200), size=3))

    @classmethod
    def get_random_rviz_color(cls):
        r, g, b = cls.generate_random_color()
        return f"{r}; {g}; {b}"

    # Sensor display generators moved to class methods
    @classmethod
    def get_sensor_color(cls, sensor_type, index=0):
        """Generate appropriate colors for different sensor types"""
        if sensor_type == 'sensor_msgs/msg/Imu':
            return "204; 51; 204"  # Consistent purple for IMU
        elif 'FootContact' in sensor_type:
            return "255; 140; 0"    # Consistent orange for FootContact
        else:
            # Generate unique colors for LaserScan and PointCloud
            r = (index * 67) % 200 + 55
            g = (index * 101) % 200 + 55
            b = (index * 173) % 200 + 55
            return f"{r}; {g}; {b}"

    class Displays:
        @classmethod
        def laser_scan(cls, topic_name, sensor_color):
            """Create LaserScan display configuration"""
            return {
                'Class': 'rviz_default_plugins/LaserScan',
                'Name': f'LaserScan: {os.path.basename(topic_name)}',
                'Enabled': True,
                'Topic': {
                    'Value': topic_name,
                    'Depth': 5,
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Best Effort',
                    'Durability Policy': 'Volatile',
                },
                'Color': sensor_color,
                'Size (m)': 0.05,
                'Style': 'Points',
                'Alpha': 1.0,
                'Decay Time': 0.0
            }

        @classmethod
        def pointcloud(cls, topic_name, sensor_color):
            """Create PointCloud2 display configuration"""
            return {
                'Class': 'rviz_default_plugins/PointCloud2',
                'Name': f'PointCloud: {os.path.basename(topic_name)}',
                'Enabled': True,
                'Topic': {
                    'Value': topic_name,
                    'Depth': 5,
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Best Effort',
                    'Durability Policy': 'Volatile',
                },
                'Color': sensor_color,
                'Size (m)': 0.03,
                'Style': 'Flat Squares',
                'Alpha': 1.0,
                'Decay Time': 0.0
            }

        @classmethod
        def pointcloud_legacy(cls, topic_name, sensor_color):
            """Create PointCloud (legacy) display configuration"""
            return {
                'Class': 'rviz_default_plugins/PointCloud',
                'Name': f'PointCloud: {os.path.basename(topic_name)}',
                'Enabled': True,
                'Topic': {
                    'Value': topic_name,
                    'Depth': 5,
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Best Effort',
                    'Durability Policy': 'Volatile',
                },
                'Color': sensor_color,
                'Size (m)': 0.03,
                'Alpha': 1.0,
                'Decay Time': 0.0
            }

        @classmethod
        def imu(cls, topic_name, sensor_color):
            """Create IMU display configuration"""
            return {
                'Class': 'rviz_default_plugins/Imu',
                'Name': f'IMU: {os.path.basename(topic_name)}',
                'Enabled': True,
                'Topic': {
                    'Value': topic_name,
                    'Depth': 5,
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Best Effort',
                    'Durability Policy': 'Volatile',
                },
                'Axes Length': 0.3,
                'Axes Radius': 0.03,
                'Color': sensor_color
            }

        @classmethod
        def footcontact(cls, topic_name, sensor_color):
            """Create FootContact display configuration"""
            return {
                'Class': 'rviz_default_plugins/Marker',
                'Name': f'FootContact: {os.path.basename(topic_name)}',
                'Enabled': True,
                'Topic': {
                    'Value': topic_name,
                    'Depth': 5,
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Best Effort',
                    'Durability Policy': 'Volatile',
                },
                'Color': sensor_color
            }

        @classmethod
        def robot_footprint(cls, topic, color):
            return {
                'Class': 'rviz_default_plugins/Polygon',
                'Name': 'Robot Footprint',
                'Enabled': True,
                'Topic': {
                    'Value': topic,
                    'Depth': 5,
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Reliable',
                    'Durability Policy': 'Volatile',
                },
                'Color': color,
                'Alpha': 1.0
            }

        @classmethod
        def local_path(cls, topic):
            return {
                'Class': 'rviz_default_plugins/Path',
                'Name': 'Local Plan',
                'Enabled': True,
                'Topic': {
                    'Value': topic,
                    'Depth': 5,
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Reliable',
                    'Durability Policy': 'Volatile',
                },
                'Color': '255; 0; 0',  # Red for local path
                'Line Width': 0.05
            }

        @classmethod
        def global_path(cls, topic, color):
            return {
                'Class': 'rviz_default_plugins/Path',
                'Name': 'Global Plan',
                'Enabled': True,
                'Topic': {
                    'Value': topic,
                    'Depth': 5,
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Reliable',
                    'Durability Policy': 'Volatile',
                },
                'Color': color,
                'Line Width': 0.05
            }

        @classmethod
        def global_costmap(cls, topic):
            return {
                'Class': 'rviz_default_plugins/Map',
                'Name': 'Global Costmap',
                'Enabled': True,
                'Topic': {
                    'Value': topic,
                    'Depth': 5,
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Reliable',
                    'Durability Policy': 'Transient Local',
                },
                'Color Scheme': 'costmap',
                'Draw Behind': False,
                'Alpha': 0.7
            }

        @classmethod
        def local_costmap(cls, topic):
            return {
                'Class': 'rviz_default_plugins/Map',
                'Name': 'Local Costmap',
                'Enabled': True,
                'Topic': {
                    'Value': topic,
                    'Depth': 5,
                    'History Policy': 'Keep Last',
                    'Reliability Policy': 'Reliable',
                    'Durability Policy': 'Transient Local',
                },
                'Color Scheme': 'costmap',
                'Draw Behind': False,
                'Alpha': 0.7
            }

        @classmethod
        def odom(cls, topic, color):
            return {
                'Class': 'rviz_default_plugins/Odometry',
                'Name': 'Odometry',
                'Enabled': True,
                'Topic': {
                    'Value': topic,
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

        @classmethod
        def robot_model(cls, robot_name):
            return {
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
