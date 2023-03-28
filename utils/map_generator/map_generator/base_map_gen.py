from abc import ABC, abstractmethod

import numpy as np


class BaseMapGenerator(ABC):
    """
    Base Map Generator as parent class for all other generators.
    """

    def __init__(self, height: int, width: int, *args, **kwargs):
        self.height = height
        self.width = width

    @abstractmethod
    def generate_grid_map(self) -> np.ndarray:
        """Generates an occupancy grid map.

        Raises:
            NotImplementedError: Must be implemented by child class.

        Returns:
            np.ndarray: 2D numpy array of the occupancy grid map with values in [0, 1].
                In shape (height, width).
        """
        raise NotImplementedError("generate_grid_map() must be implemented!")
