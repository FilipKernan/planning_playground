from abc import ABC, abstractmethod

import numpy as np


class AbstractMap(ABC):
    map: np.ndarray
    map_dimensions: tuple
    grid_size: int
    is_discrete: bool
    convex_obstacles: list[tuple]

    def __init__(self, map_path: str, resolution: tuple, origin: tuple, grid_size: int):
        self.map_path = map_path
        self.resolution = resolution
        self.origin = origin
        self.grid_size = grid_size
        self.map = self.import_map(map_path)
        pass

    @abstractmethod
    def import_map(self, map_path: str) -> np.ndarray:
        raise NotImplementedError("This method should be overridden by a subclass.")

    def get_map(self) -> np.ndarray:
        return self.map

    def get_map_dimensions(self) -> tuple:
        return self.map.shape

    def get_grid_size(self) -> int:
        return self.grid_size

    def get_origin(self) -> tuple:
        return self.origin

    def get_resolution(self) -> tuple:
        return self.resolution

    def get_convex_obstacles(self) -> list[np.ndarray]:
        return self.convex_obstacles

    @abstractmethod
    def get_map_point_in_collision(self, point: tuple, discritized_map: bool) -> bool:
        raise NotImplementedError("This method should be overridden by a subclass.")
