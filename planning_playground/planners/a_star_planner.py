import planning_playground.map.import_map as import_map
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
# todo create a better cost function
# todo create a better heuristic function

class Node:

    def __init__(self, motion_model, state, cost_equation, heuristic_equation, parent=None):
        self.state = state
        self.motion_model = motion_model
        self.parent = parent
        self.cost = 0
        self.heuristic = 0
        self.cost_equation = cost_equation
        self.heuristic_equation = heuristic_equation

    # todo - add a get neighbors function that uses the motion model to get the neighbors states
    def get_state(self):
        return self.state

    def calculate_cost(self):
        if self.parent is not None:
            self.cost = self.cost_equation(self.parent, self)
        return 0

    def calculate_heuristic(self, goal):
        self.heuristic = self.heuristic_equation(self.state, goal.state)

    def get_distance(self, other):
        return np.linalg.norm(np.array(self.state) - np.array(other.state))

    def get_heuristic(self):
        return self.heuristic
    
    def get_cost(self):
        return self.cost
    
    def get_total_cost(self):
        return self.cost + self.heuristic

    def get_neighbor_states(self):
        return self.motion_model.get_neighbor_states(self.state)
    
    def __eq__(self, other):
        return self.state == other.state
    
    def __lt__(self, other):
        return self.get_total_cost() < other.get_total_cost()
    
    def __gt__(self, other):
        return self.get_total_cost() > other.get_total_cost()
    
# The cost should live in the motion model
# The heuristic should probobly live in the motion model
   # maybe have an interface that the motion model implements for a heuristic and cost function
   # that way we can load other heuristics in the future
def calculate_cost(a, b):
    # print("calculating cost", a.get_state(), b.get_state())
    is_angle_diff = abs(a.get_state()[2] - b.get_state()[2]) > 0
    cost = a.get_cost() 
    if np.linalg.norm(np.array((a.get_state()[0], a.get_state()[1])) - np.array((b.get_state()[0], b.get_state()[1]))) > 1:
        cost += 1.2
    else:
        cost += 1
    if is_angle_diff:
        cost += 1
    return cost # todo - create a better cost function that takes into account the motion model and the cost of moving from one state to another

class PathPlanningResult:
    def __init__(self, path, timing_data, expantions):
        self.path = path
        self.timing_data = {"total": 0, "expanding": 0, "calc_cost": 0, "calc_heuristic": 0, "collision_check": 0, "checking_closed": 0, "sorting": 0, "setup": 0, "path_creation": 0, "getting_neighbors": 0, "node_creation": 0}
        self.expended_nodes = {}

def calculate_heuristic(a, b):
    return np.linalg.norm(np.array((a[0], a[1])) - np.array((b[0], b[1]))) 

class AStarPlanner:
    def __init__(self, map: import_map.Map2d, motion_model):
        self.map = map
        self.motion_model = motion_model

    # this assumes that the robot is the size of a grid cell
    # and that the robot can move in any direction - holonomic
    # todo - add the robot size as a parameter

    #Make the state space in the full map size and have the helpers that interact with the map convert the state space to the grid space

    def plan(self, start_state, goal_state, benchmark=False):
        result = PathPlanningResult([], {}, {})
        # get the current time
        start_time = time.time()
        # get the start and goal nodes
        # create the open and closed lists
        open_list = []
        closed_dict = {}
        goal = Node(self.motion_model, goal_state, calculate_cost, calculate_heuristic, None)
        start = Node(self.motion_model, start_state, calculate_cost, calculate_heuristic, None)
        start.calculate_cost()
        start.calculate_heuristic(goal)

        # add the start node to the open list
        closed_dict[start.state] = start
        # print(closed_dict)
        neighbors = start.get_neighbor_states()
        for neighbor in neighbors:
            open_list.append(Node(self.motion_model, neighbor, calculate_cost, calculate_heuristic, start))
        # print(open_list)
        end_setup_time = time.time()
        result.timing_data["setup"] = end_setup_time - start_time
        # while the open list is not empty
        open_set = set([open_node.state for open_node in open_list])
        heapq.heapify(open_list)
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
            # print("-" * 20)
            # print("expanding node", current_node.get_state())
            # print("current cost", current_node.get_cost())
            # print("current heuristic", current_node.get_heuristic())
            # print("current total cost", current_node.get_total_cost()) 
            start_getting_neighbors = time.time()
            neighbors = self.get_neighboring_nodes(current_node, goal, result.timing_data)
            result.timing_data["getting_neighbors"] += time.time() - start_getting_neighbors

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
                        heapq.heappush(open_list, neighbor)
                        open_set.add(neighbor.state)
                else: 
                    heapq.heappush(open_list, neighbor)
                    open_set.add(neighbor.state)
                end_checking_closed = time.time()
                result.timing_data["checking_closed"] += end_checking_closed - start_checking_closed
                           # add the current node to the closed list
            # print("adding to the closed dict", current_node.get_state())
            closed_dict[current_node.get_state()] = current_node
            # print("closed dict: ", closed_dict)
            # print("current heuristics", current_node.get_heuristic())
            end_expanding = time.time()
            result.timing_data["expanding"] += end_expanding - start_expanding
            # if the current node is the goal node
            # print("current node", current_node.get_state())
            # print("goal", goal.get_state())
            if current_node == goal:
                start_path = time.time()
                path = []
                print("goal found")
                current = current_node
                while current is not None:
                    print("adding node to path", current.state)
                    path.append(current.state)
                    if current.parent is not None:
                        print("parent", current.parent.state)
                    current = current.parent
                print("path", path)
                path.reverse()
                end_time = time.time()
                result.timing_data["path_creation"] = end_time - start_path
                result.timing_data["total"] = end_time - start_time
                result.path = path
                result.expended_nodes = closed_dict
                return result

        return result
    
    def get_neighboring_nodes(self, node, goal, timing_data):
        start_time = time.time()
        neighbors = node.get_neighbor_states()
        neighbor_nodes = []
        for neighbor in neighbors:
            new_node = Node(self.motion_model, neighbor, calculate_cost, calculate_heuristic, node)
            new_node.cost = node.get_cost() + self.motion_model.calc_cost(node.get_state(), new_node.get_state(), node.parent.get_state(), timing_data)
            new_node.heuristic = self.motion_model.calc_heurisitc(node.get_state(), new_node.get_state(), goal.get_state(), timing_data)
            neighbor_nodes.append(new_node)
        timing_data["getting_neighbors"] += time.time() - start_time
        return neighbor_nodes