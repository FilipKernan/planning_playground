import planning_playground.map.import_map as import_map
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time

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

def calculate_heuristic(a, b):
    return np.linalg.norm(np.array((a[0], a[1])) - np.array((b[0], b[1]))) 

class AStarPlanner:
    def __init__(self, map: import_map.Map2d, motion_model):
        self.map = map
        self.motion_model = motion_model

    # this assumes that the robot is the size of a grid cell
    # and that the robot can move in any direction - holonomic
    # todo - add the robot size as a parameter
    def plan(self, start_state, goal_state):
        start_time = time.time() 
        # get the start and goal nodes
        print("start state", start_state)
        print("goal state", goal_state)
        # create the open and closed lists
        open_list = []
        closed_list = []
        goal = Node(self.motion_model, goal_state, calculate_cost, calculate_heuristic, None)
        start = Node(self.motion_model, start_state, calculate_cost, calculate_heuristic, None)
        print("start state", start.get_state())
        print("goal state", goal.get_state())
        start.calculate_cost()
        start.calculate_heuristic(goal)
        print("start cost", start.get_cost())
        print("start heuristic", start.get_heuristic())

        # add the start node to the open list
        closed_list.append(start)
        print(closed_list)
        neighbors = start.get_neighbor_states()
        for neighbor in neighbors:
            open_list.append(Node(self.motion_model, neighbor, calculate_cost, calculate_heuristic, start))
        print(open_list)

        # while the open list is not empty
        while len(open_list) > 0:
            # get the current node
            open_list.sort()

            current_node = open_list[0]
            # get the node with the lowest cost

            # remove the current node from the open list
            open_list.pop(0)
            # print("-" * 20)
            # print("expanding node", current_node.get_state())
            # print("current cost", current_node.get_cost())
            # print("current heuristic", current_node.get_heuristic())
            # print("current total cost", current_node.get_total_cost()) 
            neighbors = current_node.get_neighbor_states()
            for neighbor in neighbors:
                # if the neighbor is in the closed list
                # we should check the cost of the neighbor vs the one in the closed list
                # if the cost of the neighbor is lower than the one in the closed list
                # we should remove the one in the closed list and add the neighbor to the open list
                if neighbor in [closed_node.state for closed_node in closed_list]:
                    continue

                # if the neighbor is not in the open list
                if neighbor not in [open_node.state for open_node in open_list]:
                    if neighbor[0] < 0 or neighbor[1] < 0 or neighbor[0] >= self.map.grid_size or neighbor[1] >= self.map.grid_size:
                        # print("neighbor is out of bounds", neighbor)
                        continue
                    # print(self.map.get_map_collision_value(neighbor))
                    if self.map.get_map_collision_value(neighbor) == 255: # this is a hold in for checking the value of the map at a specific point
                        new_node = Node(self.motion_model, neighbor, calculate_cost, calculate_heuristic, current_node)
                        new_node.calculate_cost()
                        new_node.calculate_heuristic(goal)
                        open_list.append(new_node)

            # add the current node to the closed list
            closed_list.append(current_node)

            # if the current node is the goal node
            if current_node == goal:
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
                return path, time.time() - start_time

        return None
