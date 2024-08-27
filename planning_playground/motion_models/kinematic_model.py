import planning_playground.motion_models.abstract_motion_model as abstract_motion_model
import time
from shapely.geometry import LineString, Polygon
import numpy as np
import scipy.integrate as spi
from rsplan import planner, primitives
from abc import ABC, abstractmethod

METERS_TO_PIXELS = 100


# This class models a unicycle robot with the control input being the velocity of the robot
# The state space is (x, y, theta) where x and y are the position of the robot and theta is the orientation
# The action space is (v, w) where v is the velocity of the robot and w is the angular velocity
# For now we ignore dynamics and assume the robot can instantaneously reach the desired velocity
class KinematicModel(abstract_motion_model.AbstractMotionModel):
    def __init__(
        self,
        map,
        max_velocity_linear,  # for now this is defined as pixels per second - todo is to make it meters per second
        max_angular_velocity,
        actions,
        time_step=1.0,
        is_discrete=False,
    ):
        self.time_step = time_step
        self.max_velocity_linear = max_velocity_linear
        self.max_angular_velocity = max_angular_velocity
        self.is_discrete = is_discrete
        self.actions = actions
        self.lookup_count = 0
        self.integration_dict = {}
        super().__init__(map)

    @abstractmethod
    def get_state_space(self):
        raise NotImplementedError("This method should be overridden by a subclass")

    @abstractmethod
    def get_points(self, state):
        raise NotImplementedError("This method should be overridden by a subclass")

    @abstractmethod
    def solve_ivp(self, action):
        raise NotImplementedError("This method should be overridden by a subclass")

    def integrate_forward(self, state, action, debug):
        cost = 0
        if tuple(action) not in self.integration_dict.keys():
            solution = self.solve_ivp(action)
            times = np.linspace(0, self.time_step, 3)
            interpoalted_states = solution.sol(times)
            interpoalted_states = list(
                zip(
                    interpoalted_states[0],
                    interpoalted_states[1],
                    interpoalted_states[2],
                )
            )
            for i in range(len(interpoalted_states) - 1):
                cost += np.linalg.norm(
                    np.array(
                        (
                            interpoalted_states[i][0],
                            interpoalted_states[i][1],
                            interpoalted_states[i][2] * 20,
                        )
                    )
                    - np.array(
                        (
                            interpoalted_states[i + 1][0],
                            interpoalted_states[i + 1][1],
                            interpoalted_states[i + 1][2] * 20,
                        )
                    )
                )
            if action[0] < 0:
                cost *= 2.0
            if action[1] != 0:
                cost *= 1.5
            self.integration_dict[tuple(action)] = (solution, cost, interpoalted_states)
        else:
            self.lookup_count += 1
        solution, _, interpoalted_states = self.integration_dict[tuple(action)]
        new_state_tuple = (
            (
                solution.y[0][-1] * np.cos(state[2])
                - solution.y[1][-1] * np.sin(state[2])
            )
            + state[0],
            (
                solution.y[0][-1] * np.sin(state[2])
                + solution.y[1][-1] * np.cos(state[2])
            )
            + state[1],
            self.wrap_angle_radians(solution.y[2][-1] + state[2]),
        )
        return new_state_tuple, cost, interpoalted_states

    def discretize(self, state):
        return self.get_discretized_state(state)

    def get_neighbor_states(self, state, timing_data):
        neighbors = []
        for action in self.actions:
            neighbors.append(self.integrate_forward(state, action, True)[0])
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
        footprint = []
        for i in range(0, self.wheel_base):
            x = state[0] + i * np.cos(state[2])
            y = state[1] + i * np.sin(state[2])
            footprint.append((x, y))
        return footprint

    def calc_cost(self, current_state, next_state, timing_data):
        start_time = time.time()
        cost = 0
        cost = self.simulate_motion(current_state, next_state, timing_data, 10)

        end_time = time.time()
        timing_data["calc_cost"] += end_time - start_time
        return cost

    # simulate the motion of the robot from start_state to end_state and return the states along the path
    def simulate_motion(
        self, start_state: np.array, end_state: np.array, timing_date, steps=10
    ):
        states = [start_state]
        state = start_state
        action = [0, 0]

        if np.dot(start_state[:2], end_state[:2]) < 0:
            action[0] = -self.max_velocity_linear
        elif np.dot(start_state[:2], end_state[:2]) > 0:
            action[0] = self.max_velocity_linear
        else:
            action[0] = 0

        if start_state[2] - end_state[2] < 0:
            action[1] = self.max_angular_velocity
        elif start_state[2] - end_state[2] > 0:
            action[1] = -self.max_angular_velocity
        else:
            action[1] = 0

        if self.is_discrete:
            if tuple(action) in self.integration_dict.keys():
                _, cost, _ = self.integration_dict[tuple(action)]
            else:
                _, cost, _ = self.integrate_forward(start_state, action, False)
            return cost
        else:
            cost = self.get_distance(state, end_state)
            return cost

    def calc_heuristic(self, current_state, goal, timing_data):
        # todo create better heuristic
        start_time = time.time()
        h = np.linalg.norm(
            np.array((current_state[0], current_state[1], current_state[2] * 10))
            - np.array((goal[0], goal[1], goal[2] * 10))
        )
        angle = current_state[2] - goal[2]
        # h += abs(angle) * (100 / h)
        timing_data["calc_heuristic"] += time.time() - start_time
        return h + abs(angle)

    def collision_check(self, state, timing_data):
        start_collision_check = time.time()
        height, width = self.map.get_map_dimensions()
        if state[0] < 0 or state[1] < 0 or state[0] >= width or state[1] >= height:
            end_collision_check = time.time()
            timing_data["collision_check"] += (
                end_collision_check - start_collision_check
            )
            print("out of bounds")
            return True

        for point in self.get_footprint(state):
            if self.map.get_map_point_in_collision(point, self.is_discrete):
                end_collision_check = time.time()

                timing_data["collision_check"] += (
                    end_collision_check - start_collision_check
                )
                return True

        end_collision_check = time.time()
        timing_data["collision_check"] += end_collision_check - start_collision_check
        return False

    def collision_check_between_states(self, start, end, timing_data):
        collision_checking_states = []
        collision_checking_states.append(start)
        collision_checking_states.append(end)
        for i in range(len(collision_checking_states) - 1):
            state = collision_checking_states[i]
            if self.collision_check(state, timing_data):
                return True
            start_collision_check = time.time()
            print("number of countours", len(self.map.get_convex_obstacles()))
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
        print("no collision until the end")
        return self.collision_check(end, timing_data)

    def get_distance(self, state1, state2):
        return np.linalg.norm(
            np.array((state1[0], state1[1], state1[2]))
            - np.array((state2[0], state2[1], state2[2]))
        )

    def are_states_equal(self, a, b):
        if self.is_discrete:
            discretization = self.get_state_discretization()

            a_disc = self.get_discretized_state(a)
            a = a_disc
            b_disc = self.get_discretized_state(b)
            b = b_disc
        return (
            abs(a[0] - b[0]) <= 0.5
            and abs(a[1] - b[1]) <= 0.5
            and abs(a[2] - b[2]) < 0.5
        )

    def wrap_angle_radians(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def get_discretized_state(self, state):
        discretization = self.get_state_discretization()
        # return state
        return (
            state[0] // discretization[0],
            state[1] // discretization[1],
            # state[2] // discretization[2],
            (self.wrap_angle_radians(state[2]) / discretization[2]),
        )

    def get_state_discretization(self) -> tuple:
        return (
            self.map.map_dimensions[0] // self.map.grid_size,
            self.map.map_dimensions[1] // self.map.grid_size,
            0.75,
        )
