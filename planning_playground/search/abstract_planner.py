from abc import ABC, abstractmethod

from planning_playground.map.abstract_map import AbstractMap
from planning_playground.motion_models.abstract_motion_model import AbstractMotionModel
from planning_playground.search.types import PathPlanningResult, Node


class AbstractPlanner(ABC):
    start_node: Node
    goal_node: Node
    nodes: dict[tuple, Node]
    map: AbstractMap
    motion_model: AbstractMotionModel

    def __init__(self, map: AbstractMap, motion_model: AbstractMotionModel):
        self.map = map
        self.motion_model = motion_model

    @abstractmethod
    def plan(self, start: tuple, goal: tuple, benchmark=False) -> PathPlanningResult:
        raise NotImplementedError("This method should be overridden by a subclass.")
