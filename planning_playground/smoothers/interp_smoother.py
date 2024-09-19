import scipy.interpolate as spi
import numpy as np
import matplotlib.pyplot as plt
from planning_playground.search.types import Node, PathPlanningResult
from planning_playground.motion_models.abstract_motion_model import AbstractMotionModel


class InterpSmoother:
    def __init__(
        self,
        motion_model: AbstractMotionModel,
    ):
        self.motion_model = motion_model

    def smooth_path(self, path):
        x = [state[0] for state in path]
        y = [state[1] for state in path]
        theta = [state[2] for state in path]
        t = np.linspace(0, len(path), len(path))
        x_interp_func = spi.interp1d(t, x)
        y_interp_func = spi.interp1d(t, y)
        theta_interp_func = spi.interp1d(t, theta)
        new_t = np.linspace(0, len(path), len(path) * 10)
        fig, axs = plt.subplots(1, 3, figsize=(6.5, 4))

        x_interp = x_interp_func(new_t)
        y_interp = y_interp_func(new_t)
        theta_interp = theta_interp_func(new_t)
        axs[0].plot(t, x, "o", label="data")
        axs[0].plot(new_t, x_interp, label=f"S")

        axs[1].plot(t, y, "o", label="data")
        axs[1].plot(new_t, y_interp, label=f"S")

        axs[2].plot(t, theta, "o", label="data")
        axs[2].plot(new_t, theta_interp, label=f"S")
        plt.legend()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(
            x_interp,
            y_interp,
            theta_interp,
            label="interp",
        )

        ax.plot(x, y, theta, label="Data")
        plt.legend()
        plt.show()
        inter = []
        for i in range(len(x_interp)):
            inter.append([x_interp[i], y_interp[i], theta_interp[i]])
        return [inter]
