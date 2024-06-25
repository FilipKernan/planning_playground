import planning_playground.map.import_map as import_map
from planning_playground.planners.types import Node
from planning_playground.planners.types import PathPlanningResult
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time
import heapq

# todo create viz for testing path planning
    # todo create a function that takes a map and a path and displays the path on the map
    # todo animate the robot along the path
    # todo support paths that are not defined by x,y points, but by the state space of the robot
# todo create a collision checker class
# todo create a path class



class AStarPlanner:
    def __init__(self, map: import_map.Map2d, motion_model):
        self.map = map
        self.motion_model = motion_model

    # this assumes that the robot is the size of a grid cell
    # and that the robot can move in any direction - holonomic
    # todo - add the robot size as a parameter

    #Make the state space in the full map size and have the helpers that interact with the map convert the state space to the grid space

    def plan(self, start_state, goal_state, benchmark=False):
        result = PathPlanningResult()
        # get the current time
        start_time = time.time()
        # get the start and goal nodes
        # create the open and closed lists
        open_list = []
        closed_dict = {}
        goal = Node(self.motion_model, goal_state, None)
        start = Node(self.motion_model, start_state, None)
        start.calculate_cost(result.timing_data)
        start.calculate_heuristic(goal, result.timing_data)

        # add the start node to the open list
        closed_dict[start.state] = start
        neighbors = self.get_neighboring_nodes(start, goal, result.timing_data)
        print(neighbors)
        for neighbor in neighbors:
            open_list.append(neighbor)
        end_setup_time = time.time()
        result.timing_data["setup"] = end_setup_time - start_time
        # while the open list is not empty
        open_set = set([open_node.state for open_node in open_list])
        heapq.heapify(open_list)
        print(open_list)
        while len(open_list) > 0:
            # get the current node
            start_expanding = time.time()
            current_node = heapq.heappop(open_list)
            closed_dict[current_node.get_state()] = current_node
            # get the node with the lowest cost
            # print("current node", current_node.get_state())
            # remove the current node from the open list
            try:
                open_set.remove(current_node.get_state())
            except:
                print("node not in open:", current_node.get_state())
            neighbors = self.get_neighboring_nodes(current_node, goal, result.timing_data)

            for neighbor in neighbors:
                
                start_collision_check = time.time()
                if neighbor.get_state() in open_set:
                    end_collision_check = time.time()
                    result.timing_data["collision_check"] += end_collision_check - start_collision_check
                    continue

                if self.motion_model.collision_check(neighbor.get_state(), result.timing_data):
                    continue
                # print("state is not in collision!")
                # print("neighbor", neighbor.get_state())
                # this should be a function in the motion model that takes in the map and the state of the robot, and the timeing data
                start_checking_closed = time.time()
                # if the neighbor is in the closed list
                # we should check the cost of the neighbor vs the one in the closed list
                # if the cost of the neighbor is lower than the one in the closed list
                # we should remove the one in the closed list and add the neighbor to the open list
                if neighbor.state in closed_dict:
                    closed_node = closed_dict[neighbor.state]
                    if neighbor.get_total_cost() < closed_node.get_total_cost() and closed_node != start:
                        # print("removing node from closed list", closed_node.get_state())
                        closed_dict.pop(neighbor.state)
                        closed_node.parent.remove_child(closed_node)
                        for child in closed_node.get_children():
                            child.parent = neighbor
                        closed_dict[neighbor.state] = neighbor
                else: 
                    heapq.heappush(open_list, neighbor)
                    current_node.add_child(neighbor)
                    open_set.add(neighbor.state)

                end_checking_closed = time.time()
                result.timing_data["checking_closed"] += end_checking_closed - start_checking_closed

            closed_dict[current_node.get_state()] = current_node
            end_expanding = time.time()
            result.timing_data["expanding"] += end_expanding - start_expanding
            # if the current node is the goal node
            
            if self.within_termination_bounds(current_node.get_state(), goal.get_state()):
                
                start_path = time.time()
                result.path = self.get_path(current_node)
                end_time = time.time()
                result.timing_data["path_creation"] = end_time - start_path
                result.timing_data["total"] = end_time - start_time
                result.expended_nodes = closed_dict
                result.total_cost = current_node.get_total_cost()
                
                return result

        return result

    def within_termination_bounds(self, state, goal):
        return np.linalg.norm(np.array(state[:2]) - np.array(goal[:2])) < self.motion_model.position_discretization and abs(state[2] - goal[2]) < self.motion_model.orientation_discretization 

    def rewire(self, node, neighbor, closed_dict, open_list):
        if neighbor.get_state() in closed_dict:
            closed_node = closed_dict[neighbor.get_state()]
            if neighbor.get_total_cost() < closed_node.get_total_cost():
                closed_dict.pop(neighbor.get_state())
                closed_node.parent.remove_child(closed_node)
                for child in closed_node.get_children():
                    child.parent = neighbor
                closed_dict[neighbor.get_state()] = neighbor
                heapq.heappush(open_list, neighbor)
                node.add_child(neighbor) 
                return True
        return False

    def get_neighboring_nodes(self, node, goal, timing_data):
        start_time = time.time()
        neighbors = node.get_neighbor_states()
        neighbor_nodes = []
        for neighbor in neighbors:
            new_node = Node(self.motion_model, neighbor, node)
            new_node.calculate_cost(timing_data)
            new_node.calculate_heuristic(goal, timing_data)
            neighbor_nodes.append(new_node)
        timing_data["getting_neighbors"] += time.time() - start_time
        return neighbor_nodes

    def get_path(self, node):
        path = []
        current = node

        while current is not None:
            print("adding node to path", current.state)
            path.append(current.state)
            if current.parent is not None:
                print("parent", current.parent.state)
            current = current.parent
        
        path.reverse()
        return path