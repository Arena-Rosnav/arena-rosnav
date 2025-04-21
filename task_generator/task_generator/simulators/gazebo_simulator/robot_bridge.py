class RobotBridge:
    ROBOT_CONFIGS = {
        "jackal": {
            "topics": [
                # Odometry (Gazebo -> ROS2)
                {
                    "gz_topic": "/model/{robot_name}/odometry",
                    "ros_topic": "odom",
                    "ros_type": "nav_msgs/msg/Odometry",
                    "gz_type": "gz.msgs.Odometry",
                    "direction": "["
                },
                # IMU (Gazebo -> ROS2)
                {
                    "gz_topic": "/world/default/model/{robot_name}/link/base_link/sensor/imu_sensor/imu",
                    "ros_topic": "imu/data",
                    "ros_type": "sensor_msgs/msg/Imu",
                    "gz_type": "gz.msgs.IMU",
                    "direction": "["
                },
                # Velocity command (ROS2 -> Gazebo)
                {
                    "gz_topic": "/model/{robot_name}/cmd_vel",
                    "ros_topic": "cmd_vel",
                    "ros_type": "geometry_msgs/msg/Twist",
                    "gz_type": "gz.msgs.Twist",
                    "direction": "]"
                },
                # LiDAR Scan (Gazebo -> ROS2)
                {
                    "gz_topic": "/world/default/model/{robot_name}/link/base_link/sensor/gpu_lidar/scan",
                    "ros_topic": "lidar",
                    "ros_type": "sensor_msgs/msg/LaserScan",
                    "gz_type": "gz.msgs.LaserScan",
                    "direction": "["
                },
                # LiDAR Point Cloud (Gazebo -> ROS2)
                {
                    "gz_topic": "/world/default/model/{robot_name}/link/base_link/sensor/gpu_lidar/scan/points",
                    "ros_topic": "lidar/points",
                    "ros_type": "sensor_msgs/msg/PointCloud2",
                    "gz_type": "gz.msgs.PointCloudPacked",
                    "direction": "["
                },
                # TF Data (Gazebo -> ROS2)
                {
                    "gz_topic": "/model/{robot_name}/tf",
                    "ros_topic": "/tf",
                    "ros_type": "tf2_msgs/msg/TFMessage",
                    "gz_type": "gz.msgs.Pose_V",
                    "direction": "["
                },
                {
                    "gz_topic": "/model/{robot_name}/tf_static",
                    "ros_topic": "/tf_static",
                    "ros_type": "tf2_msgs/msg/TFMessage",
                    "gz_type": "gz.msgs.Pose_V",
                    "direction": "["
                }
            ]
        },
        "turtlebot": {
            "topics": [
                # Odometry (Gazebo -> ROS2)
                {
                    "gz_topic": "/model/{robot_name}/odometry",
                    "ros_topic": "odom",
                    "ros_type": "nav_msgs/msg/Odometry",
                    "gz_type": "gz.msgs.Odometry",
                    "direction": "["
                },
                # IMU (Gazebo -> ROS2)
                {
                    "gz_topic": "/world/default/model/{robot_name}/link/base_link/sensor/imu_sensor/imu",
                    "ros_topic": "imu/data",
                    "ros_type": "sensor_msgs/msg/Imu",
                    "gz_type": "gz.msgs.IMU",
                    "direction": "["
                },
                # Velocity command (ROS2 -> Gazebo)
                {
                    "gz_topic": "/model/{robot_name}/cmd_vel",
                    "ros_topic": "cmd_vel",
                    "ros_type": "geometry_msgs/msg/Twist",
                    "gz_type": "gz.msgs.Twist",
                    "direction": "]"
                },
                # RPLIDAR Scan (Gazebo -> ROS2)
                {
                    "gz_topic": "/world/default/model/{robot_name}/link/rplidar_link/sensor/gpu_lidar/scan",
                    "ros_topic": "lidar",
                    "ros_type": "sensor_msgs/msg/LaserScan",
                    "gz_type": "gz.msgs.LaserScan",
                    "direction": "["
                },
                # RPLIDAR Point Cloud (Gazebo -> ROS2)
                {
                    "gz_topic": "/world/default/model/{robot_name}/link/rplidar_link/sensor/gpu_lidar/scan/points",
                    "ros_topic": "lidar/points",
                    "ros_type": "sensor_msgs/msg/PointCloud2",
                    "gz_type": "gz.msgs.PointCloudPacked",
                    "direction": "["
                },
                # Bumper Contact (Gazebo -> ROS2)
                {
                    "gz_topic": "/world/default/model/{robot_name}/link/bumper/sensor/bumper_contact_sensor/contact",
                    "ros_topic": "bumper/contact",
                    "ros_type": "gz.msgs.Contacts",  # Adjust based on actual ROS message type
                    "gz_type": "gz.msgs.Contacts",
                    "direction": "["
                },
                # RGB-D Camera Info (Gazebo -> ROS2)
                {
                    "gz_topic": "/world/default/model/{robot_name}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info",
                    "ros_topic": "rgbd_camera/camera_info",
                    "ros_type": "sensor_msgs/msg/CameraInfo",
                    "gz_type": "gz.msgs.CameraInfo",
                    "direction": "["
                },
                # RGB-D Depth Image (Gazebo -> ROS2)
                {
                    "gz_topic": "/world/default/model/{robot_name}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image",
                    "ros_topic": "rgbd_camera/depth_image",
                    "ros_type": "sensor_msgs/msg/Image",
                    "gz_type": "gz.msgs.Image",
                    "direction": "["
                },
                # RGB-D Image (Gazebo -> ROS2)
                {
                    "gz_topic": "/world/default/model/{robot_name}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image",
                    "ros_topic": "rgbd_camera/image",
                    "ros_type": "sensor_msgs/msg/Image",
                    "gz_type": "gz.msgs.Image",
                    "direction": "["
                },
                # RGB-D Point Cloud (Gazebo -> ROS2)
                {
                    "gz_topic": "/world/default/model/{robot_name}/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points",
                    "ros_topic": "rgbd_camera/points",
                    "ros_type": "sensor_msgs/msg/PointCloud2",
                    "gz_type": "gz.msgs.PointCloudPacked",
                    "direction": "["
                },
                # TF Data (Gazebo -> ROS2)
                {
                    "gz_topic": "/model/{robot_name}/tf",
                    "ros_topic": "/tf",
                    "ros_type": "tf2_msgs/msg/TFMessage",
                    "gz_type": "gz.msgs.Pose_V",
                    "direction": "["
                },
                {
                    "gz_topic": "/model/{robot_name}/tf_static",
                    "ros_topic": "/tf_static",
                    "ros_type": "tf2_msgs/msg/TFMessage",
                    "gz_type": "gz.msgs.Pose_V",
                    "direction": "["
                }
            ]
        }
    }