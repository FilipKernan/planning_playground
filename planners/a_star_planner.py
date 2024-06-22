import planning_playground.map.import_map as import_map
import cv2
import matplotlib.pyplot as plt
import numpy as np

# todo create viz for testing path planning
    # todo create a function that takes a map and a path and displays the path on the map
    # todo animate the robot along the path
    # todo support paths that are not defined by x,y points, but by the state space of the robot
# todo create a collision checker class
# todo create a path class
# todo create a better cost function
# todo create a better heuristic function

class Node:

    def __init__(self, state, cost_equation, heuristic_equation, parent=None):
        self.state = state
        self.parent = parent
        self.cost = 0
        self.heuristic = 0
        
        self.cost_equation = cost_equation
        self.heuristic_equation = heuristic_equation

    # todo - add a get neighbors function that uses the motion model to get the neighbors states

    def calculate_cost(self):
        if self.parent is not None:
            self.cost = self.cost_equation(self.parent, self.state)
        return 0

    def calculate_heuristic(self, goal):
        self.heuristic = self.heuristic_equation(self.state, goal)

    def get_heuristic(self):
        return self.heuristic
    
    def get_cost(self):
        return self.cost
    
    def get_total_cost(self):
        return self.cost + self.heuristic

    def __eq__(self, other):
        return self.state == other.state
    
    def __lt__(self, other):
        return self.get_total_cost() < other.get_total_cost()
    
    def __gt__(self, other):
        return self.get_total_cost() > other.get_total_cost()
    
def calculate_cost(a, b):
    return a.get_cost() + b.get_cost()

def calculate_heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

class AStarPlanner:
    def __init__(self, map: import_map.Map2d):
        self.map = map

    # this assumes that the robot is the size of a grid cell
    # and that the robot can move in any direction - holonomic
    # todo - add the robot size as a parameter
    def plan(self, start_state, goal_state):
        
        # get the start and goal nodes

        # create the open and closed lists
        open_list = []
        closed_list = []
        goal = Node(goal_state, calculate_cost, calculate_heuristic, None)
        start = Node(start_state, calculate_cost, calculate_heuristic, None)
        start.calculate_cost()
        start.calculate_heuristic(goal)
        # add the start node to the open list
        closed_list.append(start)

        neighbors = self.map.get_neighbors(start.state)
        for neighbor in neighbors:
            open_list.append(Node(neighbor, calculate_cost, calculate_heuristic, start_node))
        
        # while the open list is not empty
        while len(open_list) > 0:
            # get the current node
            open_list.sort()

            current_node = open_list[0]
            # get the node with the lowest cost

            # remove the current node from the open list
            open_list.pop(0)
            
            neighbors = self.map.get_neighbors(current_node[0])
            for neighbor in neighbors:
                # if the neighbor is in the closed list
                if neighbor in [closed_node.state for closed_node in closed_list]:
                    continue

                # if the neighbor is not in the open list
                if neighbor not in [open_node.state for open_node in open_list]:
                    if self.map.get_map_value(neighbor) < 100: # this is a hold in for checking the value of the map at a specific point
                        new_node = Node(neighbor, calculate_cost, calculate_heuristic, current_node)
                        new_node.calculate_cost()
                        new_node.calculate_heuristic(goal)
                        open_list.append(new_node)

            # add the current node to the closed list
            closed_list.append(current_node)

            # if the current node is the goal node
            if current_node == goal:
                path = []
                current = current_node
                while current is not None:
                    path.append(current.state)
                    current = current.parent
                return path.reverse()

        return None
