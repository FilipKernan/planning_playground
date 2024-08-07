import time
import sys
import heapq

import numpy as np

import planning_playground.planners.rrt_planner as rrt_planner
from planning_playground.map.abstract_map import AbstractMap
from planning_playground.motion_models.abstract_motion_model import AbstractMotionModel
from planning_playground.planners.types import Node, PathPlanningResult


# todo: cleanup
class RRTStarPlanner(rrt_planner.RRTPlanner):
    def __init__(self, map: AbstractMap, motion_model: AbstractMotionModel):
        super().__init__(map, motion_model)
        # note: dense maps require more itterations
        # idea: create anytime planner that modulates the max itterations and the radius
        # todo: radius and max itterations should be parameters
        self.radius = 100
        self.max_iter = 300
        self.path_limit = 1000  # limiting paths to 1000 nodes
        self.goal_threshold = self.radius * 2

    def plan(self, start: tuple, goal: tuple):
        result = PathPlanningResult()
        self.start_node = Node(self.motion_model, start, None)
        self.start_node.cost = 0
        self.goal_node = Node(self.motion_model, goal, None)
        self.nodes[start] = self.start_node
        start_time = time.time()
        sample_count = 0
        while sample_count < self.max_iter:
            start_expanding = time.time()
            sample_count += 1
            self.sample(result)
            result.timing_data["expanding"] += time.time() - start_expanding
            print("sample count", sample_count)

            if sample_count % 100 == 0:
                print("sample count", sample_count)

        result.path = self.get_path(self.goal_node)
        result.timing_data["total"] = time.time() - start_time
        result.expended_nodes = self.nodes
        result.total_cost = self.goal_node.get_cost()
        return result

    def sample(self, result):
        start_expanding = time.time()
        # create a sampler class base class that has a get_random_state method
        random_state = self.motion_model.sample_state(result.timing_data)
        rand_node = Node(self.motion_model, random_state)
        self.process_new_node(rand_node, result, start_expanding)

    def process_new_node(
        self, rand_node: Node, result: PathPlanningResult, start_expanding: float
    ):
        nearest, neighborhood = self.get_neighborhood(rand_node, result.timing_data)
        if nearest is None:
            print("no neighborhood found")
            result.timing_data["expanding"] += time.time() - start_expanding
            return

        rand_node.parent = nearest
        rand_node.calculate_cost(result.timing_data)

        if len(neighborhood) > 0:
            cost_to_random_node_for_each_neighbor = [
                n.get_cost()
                + self.motion_model.get_distance(n.get_state(), rand_node.get_state())
                for n in neighborhood
            ]
            if any(
                x < rand_node.get_cost() for x in cost_to_random_node_for_each_neighbor
            ):
                start_time = time.time()
                self.rewire(rand_node, neighborhood, result.timing_data)
                result.timing_data["rewiring"] += time.time() - start_time
                rand_node.calculate_cost(result.timing_data)
        self.nodes[rand_node.get_state()] = rand_node
        self.rewire_neighborhood(rand_node, neighborhood, result)
        # this needs to use the cost to get the nearest node to the goal state, fix this after fixing the neighborhood rewire
        if (
            self.motion_model.calc_cost(
                rand_node.get_state(),
                self.goal_node.get_state(),
                result.timing_data,
            )
            < self.goal_threshold
            and not self.motion_model.collision_check_between_states(
                rand_node.get_state(),
                self.goal_node.get_state(),
                result.timing_data,
            )
        ):
            self.goal_threshold = self.motion_model.calc_cost(
                rand_node.get_state(),
                self.goal_node.get_state(),
                result.timing_data,
            )
            print("new goal threshold", self.goal_threshold)
            self.goal_node.parent = rand_node
            print("goal state parent", self.goal_node.parent.get_state())

    # returns the node within the neighbor radius with the lowest cost to the target node and the list of nodes within the radius
    def get_neighborhood(self, node: Node, timing_data):
        nodes_within_radius = set()
        for expanded_node in self.nodes.values():
            distance = self.motion_model.get_distance(
                expanded_node.get_state(), node.get_state()
            )
            if distance < self.radius:
                nodes_within_radius.add(expanded_node)
        nodes_within_radius = list(nodes_within_radius)
        print(
            f"Found {len(nodes_within_radius)} nodes within radius of {node.get_state()}"
        )
        print("neighborhood", [str(n) for n in nodes_within_radius])
        nearest = super().get_nearest_node(node, timing_data)

        return nearest, nodes_within_radius

    # todo break this up if you can, but you need to put a doc string here
    # also add timing data to this
    # I think this can use the rewire node
    def rewire_neighborhood(
        self, node: Node, neighborhood: list[Node], result: PathPlanningResult
    ):
        start_time = time.time()
        neighborhood_with_new_node = neighborhood.copy()
        neighborhood_with_new_node.append(node)
        for neighbor in neighborhood:
            self.rewire(neighbor, neighborhood_with_new_node, result.timing_data)
        result.timing_data["rewiring"] += time.time() - start_time

    # rewires the node that was just sampled. This has to be a special case because the node has just been added
    # above is a weak statement, check to see if it is actually true
    def rewire(self, node: Node, neighborhood: list[Node], timing_data):
        inner_neighborhood = neighborhood.copy()
        try:
            inner_neighborhood.remove(node)
        except ValueError:
            print("node not in neighborhood")
        neighbor_cost_with_new_node = [
            (
                n.get_cost()
                + self.motion_model.calc_cost(
                    node.get_state(), n.get_state(), timing_data
                ),
                n,
            )
            for n in inner_neighborhood
        ]
        heapq.heapify(neighbor_cost_with_new_node)
        # THis loop is taking up most of the time, mostly in collision checking and calculating cost
        rewired = False
        distance_to_parent = 0
        if node.parent is not None:
            distance_to_parent = self.motion_model.calc_cost(
                node.get_state(),
                node.parent.get_state(),
                timing_data,
            )
        while len(neighbor_cost_with_new_node) > 0 and not rewired:
            x = heapq.heappop(neighbor_cost_with_new_node)
            cost_from_neighbor_to_x = self.motion_model.calc_cost(
                node.get_state(), x[1].get_state(), timing_data
            )
            print(
                f"trying to rewire {str(node)} with {str(x[1])} which has a cost of {x[0]} while the current cost is {node.get_cost()}"
            )
            if (
                (x[0] < node.get_cost() or distance_to_parent > self.radius)
                and self.can_reach_start(x[1])
                and node not in x[1].get_ancestry()
            ):
                print("trying to rewire")
                if (
                    cost_from_neighbor_to_x <= self.radius
                    and not self.motion_model.collision_check_between_states(
                        x[1].get_state(), node.get_state(), timing_data
                    )
                ):
                    try:
                        self.nodes[node.get_state()].parent = self.nodes[
                            x[1].get_state()
                        ]
                        self.nodes[node.get_state()].calculate_cost(timing_data)
                    except KeyError:
                        print(
                            "key error, this is fine as the node has just not been added to the list of expanded nodes yet"
                        )
                    node.parent = self.nodes[x[1].get_state()]
                    node.calculate_cost(timing_data)
                    print("successful rewire")
                    rewired = True
                else:
                    print("collision detected")

    def can_reach_start(self, node: Node):
        current_node = node
        count = 0
        while current_node is not None and count < self.path_limit:
            if current_node == self.start_node:
                return True
            current_node = current_node.parent
            count += 1
        return False
