import heapq
import copy
import time

import numpy as np

from planning_playground.map.abstract_map import AbstractMap
from planning_playground.motion_models.abstract_motion_model import AbstractMotionModel
import planning_playground.planners.abstract_planner as abstract_planner
from planning_playground.planners.types import Node, PathPlanningResult

# todo create viz for testing path planning
# todo create a function that takes a map and a path and displays the path on the map
# todo animate the robot along the path
# todo support paths that are not defined by x,y points, but by the state space of the robot
# todo create a collision checker class
# todo create a path class


class AStarPlanner(abstract_planner.AbstractPlanner):
    def __init__(self, map: AbstractMap, motion_model: AbstractMotionModel):
        self.map = map
        self.motion_model = motion_model
        self.open_list: list[Node] = []
        self.closed_dict: dict[tuple, Node] = {}
        self.start: Node
        self.result = PathPlanningResult()
        self.goal_node: Node
        self.collisions = 0

    # this assumes that the robot is the size of a grid cell
    # and that the robot can move in any direction - holonomic
    # todo - add the robot size as a parameter

    # Make the state space in the full map size and have the helpers that interact with the map convert the state space to the grid space
    def add_node_to_open_list(self, node: Node):
        node.calculate_cost(self.result.timing_data)
        node.calculate_heuristic(self.goal_node, self.result.timing_data)
        heapq.heappush(self.open_list, node)

    def add_node_to_closed_dict(self, node: Node):
        if node is not self.start and node.parent is not self.start:
            assert node.parent is not None
            assert node.parent.parent is not None
        node.calculate_cost(self.result.timing_data)
        node.calculate_heuristic(self.goal_node, self.result.timing_data)
        self.closed_dict[self.motion_model.get_discretized_state(node.get_state())] = (
            node
        )

    def start_up(self, start_state: tuple, goal_state: tuple):
        goal = Node(self.motion_model, goal_state, None)
        self.goal_node = goal
        self.start = Node(self.motion_model, start_state, None)
        self.start.calculate_cost(self.result.timing_data)
        self.start.calculate_heuristic(goal, self.result.timing_data)

        # add the start node to the open list
        self.closed_dict[
            self.motion_model.get_discretized_state(self.start.get_state())
        ] = self.start

    # todo - split this up into smaller functions including one that expands the node
    def plan(self, start_state, goal_state, benchmark=False):
        # get the current time
        start_time = time.time()
        # get the start and goal nodes
        self.start_up(start_state, goal_state)
        # create the open and closed lists
        neighbors = self.get_neighboring_nodes(
            self.start, self.goal_node, self.result.timing_data
        )
        print(neighbors)
        for neighbor in neighbors:
            self.open_list.append(neighbor)
        end_setup_time = time.time()
        self.result.timing_data["setup"] = end_setup_time - start_time
        # while the open list is not empty
        self.open_set = set(
            [open_node.get_discrete_state() for open_node in self.open_list]
        )
        heapq.heapify(self.open_list)
        # print(self.open_list)
        loop_count = 0
        while len(self.open_list) > 0:
            # get the current node
            # if the current node is the goal node
            print("open list", len(self.open_list))
            current_node = self.expand(self.goal_node)
            loop_count += 1
            # print("expanded_nodes", len(self.closed_dict.keys()))
            # print("current node", current_node.get_state())
            # print("goal node", self.goal_node.get_state())
            if (
                self.within_termination_bounds(
                    current_node.get_state(), self.goal_node.get_state()
                )
                or time.time() - start_time > 30
            ):
                print("goal reached")
                start_path = time.time()
                self.result.path = self.get_path(current_node)
                end_time = time.time()
                self.result.timing_data["path_creation"] = end_time - start_path
                self.result.timing_data["total"] = end_time - start_time
                self.result.expanded_nodes = self.closed_dict
                self.result.total_cost = current_node.get_total_cost()
                # for expanded_node in self.closed_dict.values():
                #     for ancestor in expanded_node.get_ancestry():
                #         assert (
                #             ancestor in self.closed_dict.values()
                #         ), f"ancestor {ancestor.get_state()} not in closed dict {list(str(x.get_state()) + '/' for x in self.closed_dict.values())} discritized state {self.motion_model.get_discretized_state(ancestor.get_state())}, closed dict keys {list(str(x) + '/' for x in self.closed_dict.keys())}"
                #         assert self.is_in_closed_dict(ancestor.get_state())
                # for expanded_node in self.result.expanded_nodes.values():
                #     for ancestor in expanded_node.get_ancestry():
                #         assert ancestor in self.result.expanded_nodes.values()
                print(("**" * 20) + str(loop_count))
                print("expanded nodes", len(self.closed_dict.keys()))
                print("expanded nodes", len(self.closed_dict.keys()))
                print("collisions", self.collisions)
                return self.result
            self.result.expanded_nodes = self.closed_dict

        return self.result

    def expand(self, goal: Node) -> Node:
        start_expanding = time.time()
        current_node = heapq.heappop(self.open_list)
        # get the node with the lowest cost
        # print("current node", current_node.get_state())
        # remove the current node from the open list
        try:
            self.open_set.remove(current_node.get_discrete_state())
        except KeyError:
            print("node not in open:", current_node.get_discrete_state())
        neighbors = self.get_neighboring_nodes(
            current_node, goal, self.result.timing_data
        )
        for neighbor in neighbors:
            assert neighbor is not None
            assert neighbor.parent is current_node

        self.add_neighbors_to_open_list(current_node, neighbors)
        self.add_node_to_closed_dict(current_node)
        end_expanding = time.time()
        self.result.timing_data["expanding"] += end_expanding - start_expanding
        return current_node

    def add_neighbors_to_open_list(self, current_node: Node, neighbors: list[Node]):
        for neighbor in neighbors:
            start_collision_check = time.time()
            # removing this for now as I do not think it is necessary
            if neighbor.get_discrete_state() in self.open_set:
                end_collision_check = time.time()
                self.result.timing_data["collision_check"] += (
                    end_collision_check - start_collision_check
                )
                continue

            if self.motion_model.collision_check(
                neighbor.get_state(), self.result.timing_data
            ):
                self.result.timing_data["collision_check"] += (
                    time.time() - start_collision_check
                )
                continue
            self.result.timing_data["collision_check"] += (
                time.time() - start_collision_check
            )
            # print("state is not in collision!")
            # print("neighbor", neighbor.get_state())
            # this should be a function in the motion model that takes in the map and the state of the robot, and the timeing data
            start_checking_closed = time.time()
            # if the neighbor is in the closed list
            # we should check the cost of the neighbor vs the one in the closed list
            # if the cost of the neighbor is lower than the one in the closed list
            # we should remove the one in the closed list and add the neighbor to the open list
            if not self.rewire(current_node, neighbor):
                heapq.heappush(self.open_list, neighbor)
                self.open_set.add(neighbor.get_discrete_state())

            end_checking_closed = time.time()
            self.result.timing_data["checking_closed"] += (
                end_checking_closed - start_checking_closed
            )

    def within_termination_bounds(self, state: tuple, goal: tuple) -> bool:
        return self.motion_model.are_states_equal(state, goal)

    def rewire(self, node: Node, neighbor: Node) -> bool:
        start_rewiring = time.time()
        discritized_state = neighbor.get_discrete_state()
        print("discritized_state", discritized_state)
        # print("closed_dict", self.closed_dict.keys())
        if discritized_state in self.closed_dict:
            self.collisions += 1
            closed_node = self.closed_dict[discritized_state]
            print("discritized_state in closed_dict")
            print(
                "neigh cost",
                node.get_cost()
                + self.motion_model.calc_cost(
                    node.get_state(), neighbor.get_state(), self.result.timing_data
                )
                + neighbor.get_heuristic(),
            )
            print("expanded cost", closed_node.get_total_cost())
            if (
                node.get_cost()
                + self.motion_model.calc_cost(
                    node.get_state(), neighbor.get_state(), self.result.timing_data
                )
                + neighbor.get_heuristic()
                < closed_node.get_total_cost()
                and closed_node != self.start
            ):
                print("trying to rewire")
                # print("removing node from closed list", closed_node.get_state())
                # self.closed_dict.pop(discritized_state)
                closed_node.parent = node
                closed_node.calculate_cost(self.result.timing_data)
                self.result.timing_data["rewiring"] += time.time() - start_rewiring
                return True
        self.result.timing_data["rewiring"] += time.time() - start_rewiring
        return False

    def get_neighboring_nodes(
        self, node: Node, goal: Node, timing_data: dict
    ) -> list[Node]:
        start_time = time.time()
        neighbors = node.get_neighbor_states(timing_data)
        neighbor_nodes = []
        for neighbor in neighbors:
            new_node = Node(self.motion_model, neighbor, node)
            new_node.calculate_cost(timing_data)
            new_node.calculate_heuristic(goal, timing_data)
            neighbor_nodes.append(new_node)
        timing_data["getting_neighbors"] += time.time() - start_time
        return neighbor_nodes

    def get_path(self, node) -> list[tuple]:
        path = []
        current = node

        while current is not None:
            print("adding node to path", current.state)
            path.append(current.state)
            if current.parent is not None:
                print("parent", current.parent.state)
            else:
                print("parent is None")
                print("current", current.state)
                print("goal", self.goal_node.state)
            current = current.parent

        path.reverse()
        if len(path) == 0 or len(path) == 1:
            print("no path found")
            return []
        return path

    def is_in_closed_dict(self, state: tuple) -> bool:
        return self.motion_model.get_discretized_state(state) in self.closed_dict.keys()
