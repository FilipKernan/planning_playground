from abc import ABC, abstractmethod

import numpy as np

from planning_playground.map.abstract_map import AbstractMap


class AbstractMotionModel(ABC):
    map: AbstractMap
    is_discrete: bool

    @abstractmethod
    def __init__(self, map: AbstractMap):
        self.map = map

    @abstractmethod
    def get_state_space(self) -> tuple:
        raise NotImplementedError("This method should be overridden by a subclass")

    @abstractmethod
    def get_neighbor_states(self, state: tuple, timing_data: dict) -> list[tuple]:
        raise NotImplementedError("This method should be overridden by a subclass")

    @abstractmethod
    def sample_state(self, timing_data: dict) -> tuple:
        raise NotImplementedError("This method should be overridden by a subclass")

    @abstractmethod
    def get_footprint(self, state: tuple) -> np.ndarray:
        raise NotImplementedError("This method should be overridden by a subclass")

    @abstractmethod
    def collision_check(self, state: tuple, timing_data: dict) -> bool:
        raise NotImplementedError("This method should be overridden by a subclass.")

    @abstractmethod
    def calc_cost(
        self, current_state: tuple, next_state: tuple, timing_data: dict
    ) -> float:
        raise NotImplementedError("This method should be overridden by a subclass")

    @abstractmethod
    def calc_heuristic(
        self, current_state: tuple, goal_state: tuple, timing_data: dict
    ) -> float:
        raise NotImplementedError("This method should be overridden by a subclass")

    @abstractmethod
    def collision_check_between_states(
        self, start: tuple, end: tuple, timing_data: dict
    ) -> bool:
        raise NotImplementedError("This method should be overridden by a subclass")

    @abstractmethod
    def get_distance(self, state1: tuple, state2: tuple) -> float:
        raise NotImplementedError("This method should be overridden by a subclass")

    @abstractmethod
    def are_states_equal(self, state1: tuple, state2: tuple) -> bool:
        raise NotImplementedError("This method should be overridden by a subclass")

    @abstractmethod
    def get_discretized_state(self, state: tuple) -> tuple:
        raise NotImplementedError("This method should be overridden by a subclass")

    @abstractmethod
    def get_state_discretization(self) -> tuple:
        raise NotImplementedError("This method should be overridden by a subclass")
