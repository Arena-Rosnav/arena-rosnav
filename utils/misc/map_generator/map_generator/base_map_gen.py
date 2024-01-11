from abc import ABC, abstractmethod
from typing import Iterable, Union

import numpy as np
import rospy
from map_generator.constants import MAP_GENERATOR_NS


class BaseMapGenerator(ABC):
    """
    Base class for map generators.

    Attributes:
        height (int): The height of the map.
        width (int): The width of the map.
        map_resolution (float): The resolution of the map.
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
        """
        Retrieves the map parameters from ROS.

        Returns:
            Iterable[Union[int, float, str]]: A tuple containing the height, width, and map resolution.
        """

        height = rospy.get_param(
            MAP_GENERATOR_NS("map_properties", "height"), self.height
        )
        width = rospy.get_param(MAP_GENERATOR_NS("map_properties", "width"), self.width)
        map_res = rospy.get_param(
            MAP_GENERATOR_NS("map_properties", "resolution"), self.map_resolution
        )
        return height, width, map_res

    @abstractmethod
    def update_params(self, height: int, width: int, map_res: float):
        """
        Updates the map parameters.

        Args:
            height (int): The new height of the map.
            width (int): The new width of the map.
            map_res (float): The new resolution of the map.
        """

        self.height = height
        self.width = width
        self.map_resolution = map_res
