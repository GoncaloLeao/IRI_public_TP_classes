"""
IRI - TP4 - Occupancy Grid base class
By: Gonçalo Leão
"""
import math
from abc import ABC

import numpy as np

from controller import LidarPoint

from typing import Union, Tuple


class OccupancyGrid(ABC):
    def __init__(self, origin: (float, float), dimensions: (int, int), resolution: float):
        self.origin: (float, float) = origin  # (x,y) real coordinate of the lower-left pixel
        self.dimensions: (int, int) = dimensions  # grid map number of (cols, row)
        self.resolution: float = resolution  # number of meters for each pixel
        self.max_coords: (float, float) = tuple(self.origin[i] + self.resolution * self.dimensions[i] for i in
                                                [0, 1])  # maximum real (x,y) coordinates inside the grid (these values are not included in the grid)

    def update_map(self, robot_tf: np.ndarray, lidar_points: [LidarPoint]) -> None:
        pass

    def real_to_grid_coords(self, coords: (float, float)) -> (int, int):
        # Note: the grid coords are [y,x], not [x,y], like real coords, so they are switched here
        return tuple(math.floor((coords[i] - self.origin[i]) / self.resolution) for i in [1, 0])

    def are_grid_coords_in_bounds(self, coords: (int, int)) -> bool:
        for i in range(2):
            if coords[i] < 0 or coords[i] >= self.dimensions[i]:
                return False
        return True
