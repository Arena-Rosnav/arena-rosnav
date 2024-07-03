import math
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


class PointCloudTransformer:
    def __init__(self, angle_min: float, angle_max: float, angle_inc: float) -> None:
        self._angle_min = angle_min
        self._angle_max = angle_max
        self._angle_inc = angle_inc

        self._default_buffer = np.zeros(
            math.ceil((self._angle_max - self._angle_min) / self._angle_inc)
        )

    def pc2_laserscan_to_laserscan(self, pointcloud: np.ndarray, buffer: np.ndarray):
        points = pointcloud.reshape(-1).view(np.complex64)
        angles = np.angle(points)
        indices = np.clip(
            (angles - self._angle_min) / self._angle_inc, 0, buffer.size - 1
        ).astype(int)
        buffer[indices] = np.abs(points)
        return buffer[1:-1]

    def pc2_pointcloud_to_laserscan(self, pointcloud: np.ndarray, buffer: np.ndarray):
        raise NotImplementedError()

    def to_laserscan(self, pointcloud: np.ndarray, buffer: np.ndarray = None):
        buffer = self._default_buffer.copy() if buffer is None else buffer.fill(0)
        method = (
            self.pc2_laserscan_to_laserscan
            if np.unique(pointcloud[:, 2]).size == 1
            else self.pc2_pointcloud_to_laserscan
        )
        return method(pointcloud, buffer)
