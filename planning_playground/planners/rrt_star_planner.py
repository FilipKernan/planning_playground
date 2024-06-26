import time

import numpy as np

import planning_playground.planners.rrt_planner as rrt_planner
from planning_playground.map.abstract_map import AbstractMap
from planning_playground.motion_models.abstract_motion_model import AbstractMotionModel
from planning_playground.planners.types import Node, PathPlanningResult


class RRTStarPlanner(rrt_planner.RRTPlanner):
    def __init__(self, map: AbstractMap, motion_model: AbstractMotionModel):
        super().__init__(map, motion_model)
        self.radius = 100
        self.rewire_threshold = 0.5

    def plan(self, start: tuple, goal: tuple):
        result = PathPlanningResult()
        self.start_node = Node(self.motion_model, start, None)
        self.goal_node = Node(self.motion_model, goal, None)
        self.nodes[start] = self.start_node
        start_time = time.time()
        sample_count = 0
        while sample_count < self.max_iter:
            start_expanding = time.time()
            # create a sampler class base class that has a get_random_state method
            random_state = self.motion_model.sample_state(result.timing_data)
            rand_node = Node(self.motion_model, random_state)
            nearest_node = self.get_nearest_node(rand_node, result.timing_data)
            if nearest_node is None:
                print("no nearest node found")
                result.timing_data["expanding"] += time.time() - start_expanding
                continue
            neighborhood = self.get_neighborhood(rand_node, result.timing_data)
            if len(neighborhood) == 0:
                print("no neighborhood found")
                result.timing_data["expanding"] += time.time() - start_expanding
                continue
            print("nearest node", nearest_node.get_state())
            print("rand node", rand_node.get_state())
            rand_node.parent = nearest_node
            rand_node.cost = rand_node.get_cost()
            nearest_node.children.append(rand_node)
            self.nodes[rand_node.get_state()] = rand_node

            print("rewiring neighborhood")
            for neighbor in neighborhood:
                self.rewire(neighbor, neighborhood, result.timing_data)
            if (
                self.motion_model.get_distance(
                    rand_node.get_state(), self.goal_node.get_state()
                )
                < self.goal_threshold
                and not self.motion_model.collision_check_along_line(
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
            print("sample count", sample_count)
            sample_count += 1

            if sample_count % 100 == 0:
                print("sample count", sample_count)

        result.path = self.get_path(self.goal_node)
        result.timing_data["total"] = time.time() - start_time
        result.expended_nodes = self.nodes
        result.total_cost = self.goal_node.get_cost()
        return result

    # returns the node within the neighbor radius with the lowest cost to the target node and the list of nodes within the radius
    def get_neighborhood(self, node: Node, timing_data):
        nodes_within_radius = []
        for expanded_node in self.nodes.values():
            distance = self.motion_model.get_distance(
                expanded_node.get_state(), node.get_state()
            )
            if distance < self.radius:
                nodes_within_radius.append(node)
        # print("closest nodes", closest_nodes)
        nodes_within_radius.append(node)
        neighborhood = {}
        for neighbor in nodes_within_radius:
            neighborhood[
                self.motion_model.get_distance(neighbor.get_state(), node.get_state())
            ] = neighbor
        print(
            f"Found {len(nodes_within_radius)} nodes within radius of {node.get_state()}"
        )
        print("neighborhood", neighborhood)
        return nodes_within_radius

    # todo: also check the children of the node one level down
    def rewire(self, node: Node, neighborhood: list[Node], timing_data):
        if node.parent is None:
            return
        start_time = time.time()
        neighborhood = neighborhood.copy()
        parent_node = node.parent
        neighborhood.remove(node)
        neighborhood.append(parent_node)
        sorted_neighborhood = sorted(
            neighborhood,
            key=lambda x: self.motion_model.calc_cost(
                node.get_state(), x.get_state(), timing_data
            ),
        )
        print(
            "sorted neighborhood",
            [neighbor.get_state() for neighbor in sorted_neighborhood],
        )
        if sorted_neighborhood[0] == node.parent or sorted_neighborhood[0] == node:
            return
        for neighbor in sorted_neighborhood:
            if not self.motion_model.collision_check_along_line(
                node.get_state(), neighbor.get_state(), timing_data
            ):
                parent_node = neighbor
                break
        node.parent.children.remove(node)
        node.parent = parent_node
        node.parent.children.append(node)
        node.calculate_cost(timing_data)
        timing_data["rewiring"] += time.time() - start_time
