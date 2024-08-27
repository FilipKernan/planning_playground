import planning_playground.motion_models.kinematic_model as kinematic_model
import time
from shapely.geometry import LineString, Polygon
import numpy as np
import scipy.integrate as spi
from rsplan import planner, primitives

METERS_TO_PIXELS = 100


# This class models a unicycle robot with the control input being the velocity of the robot
# The state space is (x, y, theta) where x and y are the position of the robot and theta is the orientation
# The action space is (v, w) where v is the velocity of the robot and w is the angular velocity
# For now we ignore dynamics and assume the robot can instantaneously reach the desired velocity
class KinematicBicycle(kinematic_model.KinematicModel):
    def __init__(
        self,
        map,
        max_velocity_linear,  # for now this is defined as pixels per second - todo is to make it meters per second
        max_angular_velocity,
        wheel_base,
        time_step=1.0,
        is_discrete=False,
    ):
        self.wheel_base = wheel_base
        actions = [
            (max_velocity_linear, 0),
            (max_velocity_linear, max_angular_velocity),
            (max_velocity_linear, -max_angular_velocity),
            (-max_velocity_linear, 0),
            (-max_velocity_linear, max_angular_velocity),
            (-max_velocity_linear, -max_angular_velocity),
        ]
        super().__init__(
            map,
            max_velocity_linear,  # for now this is defined as pixels per second - todo is to make it meters per second
            max_angular_velocity,
            actions,
            time_step,
            is_discrete,
        )

    def get_state_space(self):
        return ("x", "y", "theta")

    def get_points(self, state):
        return (int(state[0]), int(state[1]))

    def solve_ivp(self, action):
        def change_in_state(t, state, action, wheel_base):
            x, y, theta = state
            v, w = action
            return [
                v * np.cos(theta),
                v * np.sin(theta),
                (v / wheel_base) * np.tan(w),
            ]

        default_state = (0, 0, 0)
        solution = spi.solve_ivp(
            change_in_state,
            (0, self.time_step),
            (default_state[0], default_state[1], default_state[2]),
            args=[action, self.wheel_base],
            dense_output=True,
        )
        return solution

    def evaluate_path(self, path):
        # calculate the derivative of the path
        path = np.array(path)
        print(path.shape)
        x = path[:, 0]
        y = path[:, 1]
        theta = path[:, 2]
        dx_dt = np.gradient(x)
        dy_dt = np.gradient(y)
        dtheta_dt = np.gradient(theta)

        inputted_velocity = []
        inputted_steering_angle = []
        for i in range(len(path)):
            theta_i = theta[i]
            vel = dx_dt[i] / np.cos(theta_i)
            inputted_velocity.append(vel)
            inputted_steering_angle.append(
                np.arctan((dtheta_dt[i] * self.wheel_base) / vel)
            )
        inputted_acceleration = np.gradient(inputted_velocity)
        inputted_steering_angle_rate = np.gradient(inputted_steering_angle)
        return [
            (inputted_velocity, inputted_steering_angle),
            (inputted_acceleration, inputted_steering_angle_rate),
        ]
