import time
import numpy as np



class HolonomicModel:
    def __init__(self, max_linear_velocity : tuple[float, float], max_angular_velocity, map, is_discrete = True):
        self.position_discretization = map.map_dimentions[0] // map.grid_size # the distance between each point in the grid
        self.orientation_discretization = 45
        self.is_discrete = is_discrete
        self.map = map

    
    def get_points(self, state):
        # print("state to point", state[0], state[1])
        return state[0], state[1]
    
    def get_neighbor_states(self, state):
        neighbors = []
        if self.is_discrete:
            
            for x in [-self.position_discretization, 0, self.position_discretization]:
                for y in [-self.position_discretization, 0, self.position_discretization]:
                    for z in [-self.orientation_discretization, 0, self.orientation_discretization]: # for now we can only move in 45 degree increments
                        if x == 0 and y == 0 and z == 0:
                            continue
                        angle = (state[2] + z) % 360
                        neighbors.append((state[0] + x, state[1] + y, angle))
            # print("neighbors", neighbors) 

        return neighbors
    
    def get_footprint(self, state):
        return [(state[0], state[1])]
    
    def calc_cost(self, a, b, timing_data):
        start_cost = time.time()
        is_angle_diff = abs(a[2] - b[2]) > 0
        cost = 0
        if np.linalg.norm(np.array((a[0], a[1])) - np.array((b[0], b[1])) > 1):
            cost += 1.2
        else:
            cost += 1.0
        # if is_angle_diff:
        #     cost += 10
        timing_data["calc_cost"] += time.time() - start_cost
        return cost
    
    def calc_heurisitc(self, a, b, timing_data):
        start_heuristic = time.time()
        h = np.linalg.norm(np.array((a[0], a[1])) - np.array((b[0], b[1])))
        angle = a[2] - b[2]
        angle = (angle + 180) % 360 - 180
        # h += abs(angle / 180) * 0.5
        timing_data["calc_heuristic"] += time.time() - start_heuristic
        return h
    
    # returns true if there is a collision, false if there is not
    def collision_check(self, state, timing_data, descritized_map = True):
       start_collision_check = time.time()
       height, width = self.map.map_dimentions
       if state[0] < 0 or state[1] < 0 or state[0] >= width or state[1] >= height:
         # print(self.map.get_map_collision_value(neighbor))
           end_collision_check = time.time()
           timing_data["collision_check"] += end_collision_check - start_collision_check
           # print("out of bounds")
           return True
 
       for point in self.get_footprint(state):
            # print("checking if in collision: ", point)
            if self.map.get_map_point_in_collision(point, descritized_map) :
               end_collision_check = time.time()
               timing_data["collision_check"] += end_collision_check - start_collision_check
               # print("in collision")
               return True
             
       end_collision_check = time.time()
       timing_data["collision_check"] += end_collision_check - start_collision_check
       # print("not in collision")
       return False

 