

class HolonomicModel:
    def __init__(self, max_linear_velocity : tuple[float, float], max_angular_velocity):
        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.min_leaner_velocity = -1.0 * max_linear_velocity
        self.min_angular_velocity = -max_angular_velocity
        self.orientation = 0
        self.position = (0, 0)
    
    def set_linear_velocity(self, linear_velocity):
        self.linear_velocity = min(max(linear_velocity, self.min_leaner_velocity), self.max_linear_velocity) 

    def set_angular_velocity(self, angular_velocity):
        self.angular_velocity = min(max(angular_velocity, self.min_angular_velocity), self.max_angular_velocity)


    def update(self, dt):
        self.position = (self.position[0] + self.linear_velocity * dt, self.position[1])
        self.orientation = self.orientation + self.angular_velocity * dt

    def get_position(self):
        return self.position

    def get_orientation(self):
        return self.orientation
    
    def get_linear_velocity(self):
        return self.linear_velocity
    
    def get_angular_velocity(self):
        return self.angular_velocity
    
    