# generate a bezier curve from a PPoly
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.PPoly.from_spline.html#scipy.interpolate.PPoly.from_spline
# create a PPoly from data
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.CubicSpline.html#scipy.interpolate.CubicSpline

import scipy.interpolate as spi
import scipy.special
import numpy as np
from scipy.optimize import leastsq
import matplotlib.pyplot as plt
from planning_playground.search.types import Node, PathPlanningResult
from planning_playground.motion_models.abstract_motion_model import AbstractMotionModel
from itertools import islice


class BezierSmoother:
    def __init__(
        self,
        motion_model: AbstractMotionModel,
    ):
        self.motion_model = motion_model

    def chunk(self, arr_range, arr_size):
        arr_range = iter(arr_range)
        return iter(lambda: tuple(islice(arr_range, arr_size)), ())

    def B_poly(self, n, i, t):
        return scipy.special.comb(n, i) * (1 - t) ** (n - i) * t**i

    def create_segments(self, path):
        control_points = list(self.chunk(path, 3))
        new_control_points = [list(x) for x in control_points]
        for i, control_point in enumerate(new_control_points):
            print(len(control_point))
        for i, control_point in enumerate(new_control_points):
            print("-" * 100)
            print(control_point)
            if i == 0:
                continue
            print(i)
            print(new_control_points[i - 1])
            last_control_point = new_control_points[i - 1]
            new_control_points[i].insert(0, last_control_point[-1])
        print("look here")
        print(new_control_points[0])
        print(new_control_points[1])
        print(new_control_points[2])
        new_control_points[0].append(new_control_points[1][0])

        if len(new_control_points[-1]) < 4:
            new_control_points[-1].insert(0, new_control_points[-2][-2])
            new_control_points[-2].pop(-1)
        return new_control_points

    def collision_check(self, segment):
        return False

    def create_curve(self, segment, num_dimensions=2):
        degree = len(segment) - 1
        print(degree)

        def bezier(t):
            terms = []
            for i, point in enumerate(segment):
                term = np.empty(num_dimensions, dtype=float)
                for j in range(num_dimensions):
                    term[j] = (1 - t) ** (degree - i) * t**i * point[j]
                    if i != 0 and i != degree:
                        term[j] *= degree
                terms.append(term)
            return sum(terms)

        return bezier

    def interpolate_curve(self, curve, num_points):
        print("-" * 100)
        interp_points = []
        t = np.linspace(0, 1, num_points)
        for i in range(num_points):
            c = curve(t[i])
            interp_points.append(c)
        return interp_points

    def make_segments_continuous(self, segments):
        new_segments = [segments[0]]
        for i, segment in enumerate(segments[0:]):
            if i == 0:
                continue
            first = new_segments.pop(-1)
            second = segment
            connector = []
            connection_point = second[0]
            if first[-1] != connection_point:
                print("something went wrong")
                print(first[-1])
                print(connection_point)
                print(i)
                print(first)
                print(second)
            first.pop(-1)
            second.pop(0)

            connector_wrt_first = tuple(
                connection_point[x] - first[-1][x] for x in range(len(first[-1]))
            )
            second_first_wrt_connector = tuple(
                second[0][x] - connection_point[x] for x in range(len(connection_point))
            )
            new_point = tuple(
                first[-1][x] + connector_wrt_first[x] / 2
                for x in range(len(connection_point))
            )
            connector.append(new_point)
            connector.append(
                tuple(
                    first[-1][x] + 3 * connector_wrt_first[x] / 4
                    for x in range(len(connection_point))
                )
            )
            first.append(new_point)
            connector.append(
                tuple(
                    second[0][x] - 3 * second_first_wrt_connector[x] / 4
                    for x in range(len(connection_point))
                )
            )
            second_new_point = tuple(
                second[0][x] - second_first_wrt_connector[x] / 2
                for x in range(len(connection_point))
            )
            connector.append(second_new_point)
            second.insert(0, second_new_point)
            print("*" * 100)
            print("first", first)
            print("connector", connector)
            print("second", second)
            new_segments.append(first)
            new_segments.append(connector)
            new_segments.append(second)
        return new_segments

    def smooth_path(self, path):
        # split up the path into series of 4 points
        segments = self.create_segments(path)
        print(segments[-1])
        print([len(x) for x in segments])
        segments = self.make_segments_continuous(segments)
        print([len(x) for x in segments])
        print(segments[-1])
        # do collision detection of the convex hull of those 4 points
        curves_2d = []
        curves_3d = []
        for segment in segments:
            if self.collision_check(segment):
                # handle collision
                pass
            else:
                curves_2d.append(self.create_curve(segment, 2))
                curves_3d.append(self.create_curve(segment, 3))

        inter_points = []
        for curve in curves_2d:
            inter_points.extend(self.interpolate_curve(curve, 10))
        interp_points_3d = []
        for curve in curves_3d:
            interp_points_3d.extend(self.interpolate_curve(curve, 10))

        # will need to handle theta differently
        plt.plot(*zip(*inter_points), "r+", linewidth=1)
        plt.show()
        for i, point in enumerate(inter_points):
            inter_points[i] = np.concatenate((point, [0.0]))
        print(inter_points)
        # inter_points = list(*zip(inter_points))
        # interp_points_3d = list(*zip(interp_points_3d))
        return inter_points, interp_points_3d
