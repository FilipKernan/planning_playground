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
        self.radius = 100
        self.rewire_threshold = 0.5
        self.max_iter = 300
        self.path_limit = 1000  # limiting paths to 1000 nodes

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

        print("rand node", rand_node.get_state())
        rand_node.parent = nearest
        rand_node.calculate_cost(result.timing_data)
        print("rewiring neighborhood")
        neighborhood = sorted(neighborhood)
        if len(neighborhood) > 0:
            print("*" * 50)
            print("nearest node", nearest.get_state())
            print("rand node", rand_node.get_state())
            print("random node cost before", rand_node.cost)
            print("neighborhood", [str(n) for n in neighborhood])
            print(
                "cost for each of the neighbors: ", [n.get_cost() for n in neighborhood]
            )
            cost_to_random_node_for_each_neighbor = [
                n.get_cost()
                + self.motion_model.get_distance(n.get_state(), rand_node.get_state())
                for n in neighborhood
            ]
            print(
                "cost to random node for each neighbor",
                cost_to_random_node_for_each_neighbor,
            )
            if any(
                x < rand_node.get_cost() for x in cost_to_random_node_for_each_neighbor
            ):
                print("REWIRING IS REQUIRED HERE")
                self.rewire(rand_node, neighborhood, result.timing_data)
                rand_node.calculate_cost(result.timing_data)
            self.nodes[rand_node.get_state()] = rand_node
            self.rewire_neighborhood(rand_node, neighborhood, result)
            # self.rewire(neighbor, neighbor_hood_copy, result.timing_data)
            # self.re_calc_cost(neighborhood, result.timing_data)
            print("*" * 50)
        if len(neighborhood) == 1:
            return
        rand_node.cost = rand_node.get_cost()
        self.nodes[rand_node.get_state()] = rand_node
        # this needs to use the cost to get the nearest node to the goal state, fix this after fixing the neighborhood rewire
        if self.motion_model.calc_cost(
            rand_node.get_state(),
            self.goal_node.get_state(),
            result.timing_data,
        ) < self.goal_threshold and not self.motion_model.collision_check_along_line(
            rand_node.get_state(),
            self.goal_node.get_state(),
            result.timing_data,
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
        # print("closest nodes", closest_nodes)
        # nodes_within_radius.add(node)
        nodes_within_radius = list(nodes_within_radius)
        print(
            f"Found {len(nodes_within_radius)} nodes within radius of {node.get_state()}"
        )
        print("neighborhood", [str(n) for n in nodes_within_radius])
        nearest = super().get_nearest_node(node, timing_data)

        return nearest, nodes_within_radius

    def re_calc_cost(self, nodes, timing_data):
        for node in nodes:
            node.cost = None
        self.start_node.cost = 0
        for node in nodes:
            node.calculate_cost(timing_data)

    # todo break this up if you can, but you need to put a doc string here
    def rewire_neighborhood(self, node: Node, neighborhood: list[Node], result):
        neighborhood_with_new_node = neighborhood.copy()
        neighborhood_with_new_node.append(node)
        for neighbor in neighborhood:
            inner_neighborhood = neighborhood_with_new_node.copy()
            inner_neighborhood.remove(neighbor)
            neighbor_cost_with_new_node = [
                (
                    n.get_cost()
                    + self.motion_model.get_distance(
                        neighbor.get_state(), n.get_state()
                    ),
                    n,
                )
                for n in inner_neighborhood
            ]
            heapq.heapify(neighbor_cost_with_new_node)
            print(len(neighbor_cost_with_new_node))
            while len(neighbor_cost_with_new_node) > 0:
                x = heapq.heappop(neighbor_cost_with_new_node)
                print(f"neighbor {str(neighbor)}'s cost to {str(x[1])} is {str(x[0])}")
                if (
                    (
                        x[0] < neighbor.get_cost()
                        and not self.motion_model.collision_check_along_line(
                            x[1].get_state(), neighbor.get_state(), result.timing_data
                        )
                        or (
                            self.motion_model.get_distance(
                                x[1].get_state(), neighbor.get_state()
                            )
                            < self.radius
                            and (
                                neighbor.parent is not None
                                and self.motion_model.get_distance(
                                    neighbor.get_state(), neighbor.parent.get_state()
                                )
                                > self.radius
                            )
                        )  # this last condition makes sure that we do not rewire outside of the neighborhood of the node we are rewiring
                    )
                    and x[1].parent is not None
                    and x[1].parent != neighbor
                    and self.can_reach_start(x[1])
                    and neighbor not in x[1].get_ancestry()
                ):
                    print(
                        f"rewiring is required for {str(neighbor)} with the new node at {str(x[1])}"
                    )
                    print("-" * 50)
                    self.nodes[neighbor.get_state()].parent = self.nodes[
                        x[1].get_state()
                    ]
                    neighbor.parent = self.nodes[x[1].get_state()]
                    neighbor.calculate_cost(result.timing_data)
                    self.nodes[neighbor.get_state()].calculate_cost(result.timing_data)
                    print(
                        f"neighbor {str(self.nodes[neighbor.get_state()])}'s new parent is {str(x[1])}"
                    )
                    break

    # todo: right now rewring to theh node that was just created is not allowed due to a bug
    def rewire(self, node: Node, neighborhood: list[Node], timing_data):
        start_time = time.time()
        rewire_list = []
        if len(neighborhood) == 1:
            return
        if node.parent is None:
            # temp_parent = Node(self.motion_model, (300, 300, 0), None)
            # temp_parent.cost = 10
            node.parent = neighborhood[0]
            node.calculate_cost(timing_data)
        for n in neighborhood:
            if n is node:
                print(f"Node at {str(node)} is the same as {str(n)}")
                continue
            current_cost = n.get_cost()
            if np.isnan(n.get_cost()):
                current_cost = sys.float_info.max
            rewire_list.append(
                (
                    current_cost
                    + self.motion_model.get_distance(n.get_state(), node.get_state()),
                    n,
                )
            )
        print(
            f"For node at {node.get_state()} the rewire list is {[(n[0], n[1].get_state()) for n in rewire_list]}"
        )
        heapq.heapify(rewire_list)
        parent = node.parent
        while len(rewire_list) > 0:
            possible_parent = heapq.heappop(rewire_list)
            if node.get_cost() > possible_parent[0]:
                print(
                    f"Node at {str(node)}'s cost is {node.get_cost()} and is greater than {str(possible_parent[1])} with rewire cost if  {possible_parent[0]}"
                )
                if self.motion_model.collision_check_along_line(
                    node.get_state(), possible_parent[1].get_state(), timing_data
                ):
                    print(
                        f"line between {str(node)} and {str(possible_parent[1])} is in collision"
                    )
                    continue
                parent = possible_parent[1]
                node.parent = parent
                print(f"Node at {str(node)}'s new parent is {str(parent)}")
                node.calculate_cost(timing_data)

        print(f"Node at {str(node)}'s final parent is {str(parent)}")
        timing_data["rewiring"] += time.time() - start_time

    def can_reach_start(self, node: Node):
        current_node = node
        count = 0
        while current_node is not None and count < self.path_limit:
            if current_node == self.start_node:
                return True
            current_node = current_node.parent
            count += 1
        return False
