import math
import numpy as np


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
