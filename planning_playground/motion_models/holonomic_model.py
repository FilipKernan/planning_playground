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
    
    def calc_cost(self, current_state, next_state, prev_state, timing_data):
        start_cost = time.time()
        cost = 0
        if self.is_discrete:
            is_angle_diff = abs(current_state[2] - next_state[2]) > 0
            # if np.linalg.norm(np.array((a[0], a[1])) - np.array((b[0], b[1])) > 1):
                # cost += 12
            # else:
                # cost += 10
            if is_angle_diff:
                cost += 0.1
            #angle_diff_rad, angle_diff_deg = angle_between_vectors(last_vector, next_vector)
            dist = np.linalg.norm(np.array((current_state[0], current_state[1])) - np.array((next_state[0], next_state[1])))
            if dist > self.position_discretization:
                cost += 1.5 * dist
            else:
                cost += dist
            
            # cost += abs(angle_diff_rad)
            # cost *= 2.0
            timing_data["calc_cost"] += time.time() - start_cost
        return cost
    
    def calc_heurisitc(self, last_state, current_state, goal, timing_data):
        start_heuristic = time.time()
        # get the angle between last and currnet and the goal
        # last_vector = np.array((last_state[0], last_state[1])) - np.array((current_state[0], current_state[1]))
        # next_vector = np.array((current_state[0], current_state[1])) - np.array((goal[0], goal[1]))
        # angle_diff_rad, angle_diff_deg = angle_between_vectors(last_vector, next_vector)
        # scale = 1.0 + np.sin(angle_diff_rad)
        h = np.linalg.norm(np.array((current_state[0], current_state[1])) - np.array((goal[0], goal[1])))
        angle = current_state[2] - goal[2]
        angle = (angle + 180) % 360 - 180
        h += abs(angle / 180) * 2.0
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



def angle_between_vectors(a, b):
    # Convert input vectors to numpy arrays
    a = np.array(a)
    b = np.array(b)
    
    # Calculate the dot product
    dot_product = np.dot(a, b)
    
    # Calculate the magnitudes (norms) of the vectors
    norm_a = np.linalg.norm(a)
    norm_b = np.linalg.norm(b)
    
    # Calculate the cosine of the angle
    cos_theta = dot_product / (norm_a * norm_b)
    
    # Ensure the cosine value is within the valid range for arccos ([-1, 1])
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    
    # Calculate the angle in radians
    angle_radians = np.arccos(cos_theta)
    
    # Optionally, convert the angle to degrees
    angle_degrees = np.degrees(angle_radians)
    
    return angle_radians, angle_degrees

 