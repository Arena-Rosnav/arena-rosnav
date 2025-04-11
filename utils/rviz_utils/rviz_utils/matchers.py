import re

class Matcher:
    """
    Class for regex-based topic matchers
    
    """
    
    # Core matchers 
    GLOBAL_PLAN = lambda robot_name: re.compile(f"/task_generator_node/{robot_name}/plan$|/{robot_name}/plan$")
    LASER_SCAN = lambda robot_name: re.compile(f"/task_generator_node/{robot_name}/lidar$|/{robot_name}/lidar$|/task_generator_node/{robot_name}/scan$|/{robot_name}/scan$")
    GLOBAL_COSTMAP = lambda robot_name: re.compile(f"/task_generator_node/{robot_name}/global_costmap/costmap$|/{robot_name}/global_costmap/costmap$")
    LOCAL_COSTMAP = lambda robot_name: re.compile(f"/task_generator_node/{robot_name}/local_costmap/costmap$|/{robot_name}/local_costmap/costmap$")
    GOAL = lambda robot_name: re.compile(f"/task_generator_node/{robot_name}/goal$|/{robot_name}/goal$|/task_generator_node/{robot_name}/goal_pose$|/{robot_name}/goal_pose$")
    MODEL = lambda robot_name: re.compile(f"/task_generator_node/{robot_name}/robot_description$|/{robot_name}/robot_description$")
    
    # Sensor matchers (LaserScan, PointCloud, IMU, RGBD)
    POINT_CLOUD = lambda robot_name: re.compile(f"/task_generator_node/{robot_name}/lidar/points$|/{robot_name}/lidar/points$")
    IMAGE = lambda robot_name: re.compile(f"/task_generator_node/{robot_name}/image$|/{robot_name}/image$|/task_generator_node/{robot_name}/camera/image$|/{robot_name}/camera/image$")
    IMU = lambda robot_name: re.compile(f"/task_generator_node/{robot_name}/imu/data$|/{robot_name}/imu/data$")
    
    # Odometry matcher
    ODOMETRY = lambda robot_name: re.compile(f"/task_generator_node/{robot_name}/odom$|/{robot_name}/odom$")