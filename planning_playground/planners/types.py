import numpy as np
import time



class Node:

    def __init__(self, motion_model, state, parent=None):
        self.state = state
        self.motion_model = motion_model
        self.parent = parent
        self.cost = None
        self.heuristic = None
        self.children = []

    # todo - add a get neighbors function that uses the motion model to get the neighbors states
    def get_state(self):
        return self.state

    def calculate_cost(self, timing_data):
        if self.parent is not None:
            self.cost = self.parent.get_cost() + self.cost_eq(self.parent, self, timing_data)
            return self.cost
        return 0

    def calculate_heuristic(self, goal, timing_data):
        self.heuristic = self.heuristic_eq(goal, timing_data)

    def get_distance(self, other):
        return np.linalg.norm(np.array(self.state) - np.array(other.state))

    def get_heuristic(self):
        if self.heuristic is None:
            timing_data = {"calc_heuristic": 0}
            return 0
        return self.heuristic
    
    def get_cost(self):
        if self.cost is None:
            timing_data = {"calc_cost": 0} # this is a dummy timing data
            return self.calculate_cost(timing_data)
        return self.cost
    
    def get_total_cost(self):
        return self.get_cost() + self.get_heuristic()

    def get_neighbor_states(self):
        return self.motion_model.get_neighbor_states(self.state)
    
    def get_children(self):
        return self.children
    
    def add_child(self, child):
        self.children.append(child)

    def remove_child(self, child):
        self.children.remove(child)
        child.parent = None
        return child
    
    def __eq__(self, other):
        return self.state == other.state
    
    def __lt__(self, other):
        return self.get_total_cost() < other.get_total_cost()
    
    def __gt__(self, other):
        return self.get_total_cost() > other.get_total_cost()
    
    def cost_eq(self, parent, child, timing_data):
        return self.motion_model.calc_cost(parent.get_state(), child.get_state(),  timing_data)

    def heuristic_eq(self,  goal_state, timing_data):
        return self.motion_model.calc_heurisitc(self.get_state(), goal_state.get_state(), timing_data)

class PathPlanningResult:
    def __init__(self):
        self.path = []
        self.timing_data = {"total": 0, "expanding": 0, "calc_cost": 0, "calc_heuristic": 0, "collision_check": 0, "checking_closed": 0, "sorting": 0, "setup": 0, "path_creation": 0, "getting_neighbors": 0, "node_creation": 0, "sampling": 0}
        self.expended_nodes = {}
        self.total_cost = 0