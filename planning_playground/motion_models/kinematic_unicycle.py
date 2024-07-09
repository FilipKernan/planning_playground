import planning_playground.motion_models.abstract_motion_model as abstract_motion_model
import time
from shapely.geometry import LineString, Polygon
import numpy as np

METERS_TO_PIXELS = 100


# This class models a unicycle robot with the control input being the velocity of the robot
# The state space is (x, y, theta) where x and y are the position of the robot and theta is the orientation
# The action space is (v, w) where v is the velocity of the robot and w is the angular velocity
# For now we ignore dynamics and assume the robot can instantaneously reach the desired velocity
class KinematicUnicycle(abstract_motion_model.AbstractMotionModel):
    def __init__(
        self,
        map,
        max_velocity_linear,  # for now this is defined as pixels per second - todo is to make it meters per second
        max_angular_velocity,
        time_step=0.5,
        integration_steps=10,
    ):
        self.time_step = time_step
        self.max_velocity_linear = max_velocity_linear
        self.max_angular_velocity = max_angular_velocity
        self.integration_steps = integration_steps
        super().__init__(map)

    def get_state_space(self):
        return ("x", "y", "theta")

    def integrate_forward(self, state, action):
        np_state = np.array(state)
        np_action = np.array(action[0], 0, action[1])
        for i in range(self.integration_steps):
            angle_transform = np.array(
                [
                    [np.cos(np_state[2]), -np.sin(np_state[2]), 0],
                    [np.sin(np_state[2]), np.cos(np_state[2]), 0],
                    [0, 0, 1],
                ]
            )
            np_state += (angle_transform @ np_action) * self.time_step
        new_state_tuple = (np_state[0], np_state[1], np_state[2])
        return new_state_tuple

    def get_neighbor_states(self, state, timing_data):
        neighbors = []
        actions = [
            (self.max_velocity_linear, 0),
            (self.max_velocity_linear, self.max_angular_velocity),
            (self.max_velocity_linear, -self.max_angular_velocity),
            (-self.max_velocity_linear, 0),
            (-self.max_velocity_linear, self.max_angular_velocity),
            (-self.max_velocity_linear, -self.max_angular_velocity),
        ]
        for action in actions:
            neighbors.append(self.integrate_forward(state, action))
        return neighbors

    def sample_state(self, timing_data, bounds=None, sample_method="uniform"):
        state = (0, 0, 0)
        if sample_method == "uniform":
            if bounds is not None:
                state = (
                    np.random.uniform(bounds[0], bounds[1]),
                    np.random.uniform(bounds[2], bounds[3]),
                    np.random.uniform(0, 2 * np.pi),
                )
                return state

            state = (
                np.random.uniform(0, self.map.map_dimensions[0]),
                np.random.uniform(0, self.map.map_dimensions[1]),
                np.random.uniform(0, 2 * np.pi),
            )
        else:
            print("Invalid sample method, returning zero state")
            state = (0, 0, 0)
        return state

    def get_footprint(self, state):
        return [(state[0], state[1])]

    # calculate the arc length of the circle that the robot will travel along
    def calc_cost(self, current_state, next_state, timing_data):
        cost = 0
        states = self.simulate_motion(current_state, next_state, timing_data, 10)
        for i in range(len(states) - 1):
            cost += np.linalg.norm(
                np.array(states[i][:2]) - np.array(states[i + 1][:2])
            )
        return cost

    # simulate the motion of the robot from start_state to end_state and return the states along the path
    def simulate_motion(
        self, start_state: np.array, end_state: np.array, timing_date, steps=10
    ):
        states = [start_state]
        state = start_state
        action = (0, 0)

        if start_state[:2].dot(end_state[:2]) < 0:
            action[0] = -self.max_velocity_linear
        elif start_state[:2].dot(end_state[:2]) > 0:
            action[0] = self.max_velocity_linear
        else:
            action[0] = 0

        if start_state[2] - end_state[2] < 0:
            action[1] = self.max_angular_velocity
        elif start_state[2] - end_state[2] > 0:
            action[1] = -self.max_angular_velocity
        else:
            action[1] = 0

        for i in range(steps):
            state = self.integrate_forward(state, action)
            states.append(state)

        return states

    def collision_check(self, state, timing_data):
        start_collision_check = time.time()
        height, width = self.map.get_map_dimensions()
        if state[0] < 0 or state[1] < 0 or state[0] >= width or state[1] >= height:
            end_collision_check = time.time()
            timing_data["collision_check"] += (
                end_collision_check - start_collision_check
            )
            return True

        for point in self.get_footprint(state):
            # print("checking if in collision: ", point)
            if self.map.get_map_point_in_collision(point, self.map.discretized_map):
                end_collision_check = time.time()
                timing_data["collision_check"] += (
                    end_collision_check - start_collision_check
                )
                # print("in collision")
                return True

        end_collision_check = time.time()
        timing_data["collision_check"] += end_collision_check - start_collision_check
        # print("not in collision")
        return False

    def collision_check_between_states(self, start, end, timing_data):
        states = self.simulate_motion(
            start, end, timing_data, int(abs(self.get_distance(start, end)))
        )
        collision_checking_states = [states[0]]
        necessary_distance_m = 0.01 * METERS_TO_PIXELS
        # if the map is discrete, the necessary distance is the grid size
        if self.map.is_discrete:
            necessary_distance_m = self.map.grid_size
        for state in states:
            if (
                np.abs(np.linalg.norm(state[:2], collision_checking_states[-1][:2]))
                > necessary_distance_m
            ):
                # if the distance between the current state and the last state checked is greater than the necessary distance
                collision_checking_states.append(state)
        for i in range(len(collision_checking_states) - 1):
            state = collision_checking_states[i]
            if self.collision_check(state, timing_data):
                return True
            start_collision_check = time.time()
            contours = self.map.get_convex_obstacles()
            for contour in contours:
                next_state = collision_checking_states[i + 1]
                contour = np.squeeze(contour)
                polygon = Polygon(contour)
                if polygon.intersects(LineString([state, next_state])):
                    timing_data["collision_check"] += (
                        time.time() - start_collision_check
                    )
                    return True

    def get_distance(self, state1, state2):
        return np.linalg.norm(
            np.array((state1[0], state1[1])) - np.array((state2[0], state2[1]))
        )
