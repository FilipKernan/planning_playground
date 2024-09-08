import scipy.interpolate as spi
import numpy as np
import matplotlib.pyplot as plt
from planning_playground.search.types import Node, PathPlanningResult
from planning_playground.motion_models.abstract_motion_model import AbstractMotionModel


class SplineSmoother:
    def __init__(
        self,
        motion_model: AbstractMotionModel,
    ):
        self.motion_model = motion_model

    def smooth_path(self, path):
        values = []
        for i in range(len(path[0])):
            values.append([state[i] for state in path])
        t = np.linspace(0, len(path), len(path))
        # need to handle theta differently as it can roll over
        tck, u = spi.splprep(
            values,
            s=len(path) * 2,
        )
        overall_spline = spi.spalde(u, tck)

        plotted_values = []
        for i in range(len(overall_spline)):
            plotted_values.append([d[0] for d in overall_spline[i]])
        fig = plt.figure()
        if len(plotted_values) >= 3:
            ax = fig.add_subplot(111, projection="3d")
            ax.plot(
                plotted_values[0],
                plotted_values[1],
                plotted_values[2],
                label="3d spline",
            )
            ax.plot(values[0], values[1], values[2], label="Data")
        else:
            ax = fig.add_subplot(111)
            ax.plot(
                plotted_values[0],
                plotted_values[1],
                label="spline",
            )
            ax.plot(values[0], values[1], label="Data")
        plt.legend()
        fig, axs = plt.subplots(1, len(values), figsize=(6.5, 4))

        for i, value in enumerate(values):
            axs[i].plot(t, value, "o", label="data")
            for j in range(len(overall_spline[i][0]) - 1):
                axs[i].plot(t, [d[j] for d in overall_spline[i]], label=f"S^{j}")

        plt.legend()
        plt.show()
        overall_spline_base = []
        for i in range(len(overall_spline)):
            overall_spline_base.append([d[0] for d in overall_spline[i]])
        new_spline_base = []
        for i in range(len(overall_spline_base[0])):
            new_spline_base.append(tuple(d[i] for d in overall_spline_base))
        return [
            new_spline_base,
        ]
