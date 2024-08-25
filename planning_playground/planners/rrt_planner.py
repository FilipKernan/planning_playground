import time

import numpy as np

import planning_playground.planners.abstract_planner as abstract_planner
from planning_playground.motion_models.holonomic_model import AbstractMotionModel
from planning_playground.planners.types import Node, PathPlanningResult
from planning_playground.map.abstract_map import AbstractMap
from planning_playground.planners.types import Node


class RRTPlanner(abstract_planner.AbstractPlanner):
    def __init__(self, map: AbstractMap, motion_model: AbstractMotionModel):
        self.map = map
        self.motion_model = motion_model
        self.nodes = {}
        self.start_node = None
        self.goal = None
        self.goal_node = None
        self.max_iter = 300
        self.goal_threshold = 500

    def plan(self, start, goal):
        result = PathPlanningResult()
        self.start_node = Node(self.motion_model, start, None)
        self.goal_node = Node(self.motion_model, goal, None)
        self.nodes[start] = self.start_node
        start_time = time.time()
        sample_count = 0
        while sample_count < self.max_iter:
            start_expanding = time.time()
            sample_count += 1
            # create a sampler class base class that has a get_random_state method
            random_state = self.motion_model.sample_state(result.timing_data)
            rand_node = Node(self.motion_model, random_state)
            nearest_node = self.get_nearest_node(rand_node, result.timing_data)
            if nearest_node is None:
                result.timing_data["expanding"] += time.time() - start_expanding
                continue
            rand_node.parent = nearest_node
            rand_node.cost = rand_node.get_cost()
            nearest_node.children.append(rand_node)
            self.nodes[rand_node.get_state()] = rand_node

            if (
                self.motion_model.get_distance(
                    rand_node.get_state(), self.goal_node.get_state()
                )
                < self.goal_threshold
                and not self.motion_model.collision_check_between_states(
                    rand_node.get_state(),
                    self.goal_node.get_state(),
                    result.timing_data,
                )
            ):
                self.goal_threshold = self.motion_model.get_distance(
                    rand_node.get_state(), self.goal_node.get_state()
                )
                print("new goal threshold", self.goal_threshold)
                self.goal_node.parent = rand_node
                print("goal state parent", self.goal_node.parent.get_state())

            result.timing_data["expanding"] += time.time() - start_expanding

            if sample_count % 100 == 0:
                print("sample count", sample_count)

        result.path = self.get_path(self.goal_node)
        result.timing_data["total"] = time.time() - start_time
        result.expanded_nodes = self.nodes
        result.total_cost = self.goal_node.get_cost()
        return result

    def get_nearest_node(self, node: Node, timing_data):
        start_time = time.time()
        nearest_node = None
        nearest_dist = np.inf
        for n in self.nodes.values():
            dist = self.motion_model.get_distance(n.get_state(), node.get_state())
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_node = n
        if self.motion_model.collision_check_between_states(
            nearest_node.get_state(), node.get_state(), timing_data
        ):
            nearest_node = None
        timing_data["getting_neighbors"] = time.time() - start_time
        return nearest_node

    def get_path(self, node):
        path = []
        current_node: Node = node
        count = 0

        while current_node is not None and count < 1000:
            path.append(current_node.get_state())
            current_node = current_node.parent

            if current_node is not None and current_node.state == self.start_node.state:
                path.append(current_node.state)
                break
            count += 1

        path.reverse()
        if len(path) == 0 or len(path) == 1:
            print("no path found")
            return None
        return path
