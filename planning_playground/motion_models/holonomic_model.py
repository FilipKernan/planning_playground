



class HolonomicModel:
    def __init__(self, max_linear_velocity : tuple[float, float], max_angular_velocity):
        self.position_discretization = 1
        self.orientation_discretization = 45

    
    def get_points(self, state):
        print("state to point", state[0], state[1])
        return state[0], state[1]
    
    def get_neighbor_states(self, state):
        neighbors = []
        for x in [-self.position_discretization, 0, self.position_discretization]:
            for y in [-self.position_discretization, 0, self.position_discretization]:
                for z in [-self.orientation_discretization, 0, self.orientation_discretization]: # for now we can only move in 45 degree increments
                    if x == 0 and y == 0 and z == 0:
                        continue
                    angle = max(0, min(360, state[2] + z))
                    neighbors.append((state[0] + x, state[1] + y, angle))
        return neighbors