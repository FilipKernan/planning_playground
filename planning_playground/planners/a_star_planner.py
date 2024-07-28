import heapq
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

    # this assumes that the robot is the size of a grid cell
    # and that the robot can move in any direction - holonomic
    # todo - add the robot size as a parameter

    # Make the state space in the full map size and have the helpers that interact with the map convert the state space to the grid space
    def add_node_to_open_list(self, node: Node):
        node.calculate_cost(self.result.timing_data)
        node.calculate_heuristic(self.goal_node, self.result.timing_data)
        heapq.heappush(self.open_list, node)

    def add_node_to_closed_dict(self, node: Node):
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
        # create the open and closed lists
        neighbors = self.get_neighboring_nodes(
            self.start, goal, self.result.timing_data
        )
        print(neighbors)
        for neighbor in neighbors:
            self.open_list.append(neighbor)
        end_setup_time = time.time()
        self.result.timing_data["setup"] = end_setup_time - start_time
        # while the open list is not empty
        # self.open_set = set([open_node.state for open_node in self.open_list])
        heapq.heapify(self.open_list)
        print(self.open_list)
        while len(self.open_list) > 0:
            # get the current node
            # if the current node is the goal node

            current_node = self.expand(goal)

            if self.within_termination_bounds(
                current_node.get_state(), goal.get_state()
            ):
                start_path = time.time()
                self.result.path = self.get_path(current_node)
                end_time = time.time()
                self.result.timing_data["path_creation"] = end_time - start_path
                self.result.timing_data["total"] = end_time - start_time
                self.result.expended_nodes = self.closed_dict
                self.result.total_cost = current_node.get_total_cost()

                return self.result

        return self.result

    def expand(self, goal: Node) -> Node:
        start_expanding = time.time()
        current_node = heapq.heappop(self.open_list)
        self.closed_dict[current_node.get_state()] = current_node
        # get the node with the lowest cost
        # print("current node", current_node.get_state())
        # remove the current node from the open list
        # try:
        #     self.open_set.remove(current_node.get_state())
        # except KeyError:
        #     print("node not in open:", current_node.get_state())
        neighbors = self.get_neighboring_nodes(
            current_node, goal, self.result.timing_data
        )

        self.add_neighbors_to_open_list(current_node, neighbors)
        self.closed_dict[
            self.motion_model.get_discretized_state(current_node.get_state())
        ] = current_node
        end_expanding = time.time()
        self.result.timing_data["expanding"] += end_expanding - start_expanding
        return current_node

    def add_neighbors_to_open_list(self, current_node: Node, neighbors: list[Node]):
        for neighbor in neighbors:
            start_collision_check = time.time()
            # removing this for now as I do not think it is necessary
            # if neighbor.get_state() in self.open_set:
            #     end_collision_check = time.time()
            #     self.result.timing_data["collision_check"] += (
            #         end_collision_check - start_collision_check
            #     )
            #     continue

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
            if self.rewire(current_node, neighbor):
                continue
            else:
                heapq.heappush(self.open_list, neighbor)
                current_node.add_child(neighbor)
                # self.open_set.add(neighbor.state)

            end_checking_closed = time.time()
            self.result.timing_data["checking_closed"] += (
                end_checking_closed - start_checking_closed
            )

    def within_termination_bounds(self, state: tuple, goal: tuple) -> bool:
        return self.motion_model.are_states_equal(state, goal)

    def rewire(self, node: Node, neighbor: Node) -> bool:
        discritized_state = self.motion_model.get_discretized_state(
            neighbor.get_state()
        )
        print("discritized_state", discritized_state)
        print("closed_dict", self.closed_dict.keys())
        if discritized_state in self.closed_dict:
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
                closed_node.parent.remove_child(closed_node)  # type: ignore
                node.add_child(closed_node)
                closed_node.parent = node
                closed_node.calculate_cost(self.result.timing_data)
                return True
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
            current = current.parent

        path.reverse()
        if len(path) == 0 or len(path) == 1:
            print("no path found")
            return []
        return path

    def is_in_closed_dict(self, state: tuple) -> bool:
        return self.motion_model.get_discretized_state(state) in self.closed_dict
