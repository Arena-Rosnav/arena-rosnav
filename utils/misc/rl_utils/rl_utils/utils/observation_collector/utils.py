from typing import Tuple

import numpy as np
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion

DistanceToGoal = float
AngleToGoal = float


def get_goal_pose_in_robot_frame(
    goal_pos: Pose2D, robot_pos: Pose2D
) -> Tuple[DistanceToGoal, AngleToGoal]:
    y_relative = goal_pos.y - robot_pos.y
    x_relative = goal_pos.x - robot_pos.x
    rho = (x_relative**2 + y_relative**2) ** 0.5
    theta = (np.arctan2(y_relative, x_relative) - robot_pos.theta + 4 * np.pi) % (
        2 * np.pi
    ) - np.pi
    return rho, theta


def pose3d_to_pose2d(pose3d) -> Pose2D:
    pose2d = Pose2D()
    pose2d.x = pose3d.position.x
    pose2d.y = pose3d.position.y
    quaternion = (
        pose3d.orientation.x,
        pose3d.orientation.y,
        pose3d.orientation.z,
        pose3d.orientation.w,
    )
    euler = euler_from_quaternion(quaternion)
    yaw = euler[2]
    pose2d.theta = yaw
    return pose2d


def false_params(**kwargs):
    false_params = []
    for key, val in kwargs.items():
        if not val:
            false_params.append(key)
    return false_params
