from abc import ABC, abstractmethod
from typing import Iterable, Union

import numpy as np
import rospy


class BaseMapGenerator(ABC):
    """
    Base Map Generator as parent class for all other generators.
    """

    def __init__(self, height: int, width: int, map_resolution: float, *args, **kwargs):
        self.height = height
        self.width = width
        self.map_resolution = map_resolution

    @abstractmethod
    def generate_grid_map(self) -> np.ndarray:
        """Updates parameters (retrieved from ROS) and generates a grid map.

        Raises:
            NotImplementedError: Must be implemented by child class.

        Returns:
            np.ndarray: 2D numpy array of the occupancy grid map with values in [0, 1].
                In shape (height, width).
        """
        self.update_params(*self.retrieve_params())

    @abstractmethod
    def retrieve_params(self) -> Iterable[Union[int, float, str]]:
        """Retrieve parameters from ROS parameter server."""
        height = rospy.get_param("/map_properties/height", self.height)
        width = rospy.get_param("/map_properties/width", self.width)
        map_res = rospy.get_param("/map_properties/resolution", self.map_resolution)
        return height, width, map_res

    @abstractmethod
    def update_params(self, height: int, width: int, map_res: float):
        """Update object parameters with new values."""
        self.height = height
        self.width = width
        self.map_resolution = map_res
