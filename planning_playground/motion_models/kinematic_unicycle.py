import planning_playground.motion_models.abstract_motion_model as abstract_motion_model
import time
from shapely.geometry import LineString, Polygon
import numpy as np
import scipy.integrate as spi

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
        time_step=1.0,
        is_discrete=False,
    ):
        self.time_step = time_step
        self.max_velocity_linear = max_velocity_linear
        self.max_angular_velocity = max_angular_velocity
        self.is_discrete = is_discrete
        self.actions = [
            (self.max_velocity_linear, 0),
            (self.max_velocity_linear, self.max_angular_velocity),
            (self.max_velocity_linear, -self.max_angular_velocity),
            (-self.max_velocity_linear, 0),
            (-self.max_velocity_linear, self.max_angular_velocity),
            (-self.max_velocity_linear, -self.max_angular_velocity),
            (0, self.max_angular_velocity),
            (0, -self.max_angular_velocity),
        ]
        self.lookup_count = 0
        self.integration_dict = {}
        super().__init__(map)

    def get_state_space(self):
        return ("x", "y", "theta")

    def get_points(self, state):
        return (int(state[0]), int(state[1]))

    def integrate_forward(self, state, action, debug):
        def change_in_state(t, state, action):
            x, y, theta = state
            v, w = action
            # Todo: the problem is here and only seems to exist when going straight
            return [
                v * np.cos(theta),
                v * np.sin(theta),
                w,
            ]

        if tuple(action) not in self.integration_dict.keys():
            default_state = (0, 0, 0)
            solution = spi.solve_ivp(
                change_in_state,
                (0, self.time_step),
                (default_state[0], default_state[1], default_state[2]),
                args=[action],
            )
            cost = 0
            for i in range(len(solution.t) - 1):
                cost += np.linalg.norm(
                    np.array(
                        (solution.y[0][i], solution.y[1][i], solution.y[2][i] * 10)
                    )
                    - np.array(
                        (
                            solution.y[0][i + 1],
                            solution.y[1][i + 1],
                            solution.y[2][i + 1] * 10,
                        )
                    )
                )
            self.integration_dict[tuple(action)] = (solution, cost)
        else:
            self.lookup_count += 1
            # print("lookup count", self.lookup_count)
        solution, _ = self.integration_dict[tuple(action)]
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
            wrap_angle_radians(solution.y[2][-1] + state[2]),
        )
        return new_state_tuple, solution

    def discretize(self, state):
        return self.get_discretized_state(state)

    def get_neighbor_states(self, state, timing_data):
        neighbors = []
        # print("getting neighbors")
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
        return [(state[0], state[1])]

    def calc_cost(self, current_state, next_state, timing_data):
        start_time = time.time()
        cost = 0
        _, cost = self.simulate_motion(current_state, next_state, timing_data, 10)

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

        solution, cost = self.integration_dict[tuple(action)]
        return solution, cost

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
            return True

        for point in self.get_footprint(state):
            # print("checking if in collision: ", point)
            if self.map.get_map_point_in_collision(point, self.is_discrete):
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
        )[0].y
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

    def are_states_equal(self, a, b):
        print(f"a state {a}, b state {b}")
        if self.is_discrete:
            discretization = self.get_state_discretization()

            # print("I am discrete")
            # print("discretization", discretization)
            a_disc = self.get_discretized_state(a)
            a = a_disc
            # print("a", a)
            b_disc = self.get_discretized_state(b)
            b = b_disc
            # print("b", b)
        print(
            f"differences { abs(a[0] - b[0])  }, { abs(a[1] - b[1])  } , { abs(a[2] - b[2]) }"
        )
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
            0.25,
        )


def wrap_angle_radians(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
