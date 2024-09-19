import cv2
from matplotlib import pyplot as plt
import numpy as np

import planning_playground.map.import_map as import_map
import planning_playground.motion_models.holonomic_model as holonomic_model
import planning_playground.motion_models.kinematic_unicycle as kinematic_unicycle
import planning_playground.motion_models.kinematic_bicycle as kinematic_bicycle
import planning_playground.viz.viz_plan as viz_plan
from planning_playground.search.a_star_planner import AStarPlanner
from planning_playground.search.rrt_planner import RRTPlanner
from planning_playground.search.rrt_star_planner import RRTStarPlanner
from planning_playground.search.types import PathPlanningResult
from planning_playground.smoothers.spline_smoother import SplineSmoother
from planning_playground.smoothers.interp_smoother import InterpSmoother
from planning_playground.smoothers.bezier_smoother import BezierSmoother

# create planner builder class
# should take in a map, motion model, heruistic, and planner type
# should have a plan method that takes in a start and goal state
# should return a PathPlanningResult

# create a motion model builder class
# should take in a map, and a motion model type, and the state space

global start
start = (150, 75, 0.0)
global goal
goal = (150, 640, 0.0)


def main(debug=False):
    # Create a map
    print("creating map")
    # map = import_map.Map2d("planning_playground/map/map_dense750.png", grid_size=60)
    map = import_map.Map2d(
        "planning_playground/map/map_dense172.png",
        grid_size=50,
    )
    print("created map")
    # todo: make a ui for chousing the start and goal
    # cv2.imshow("map", map.map)
    # cv2.setMouseCallback("map", set_start_goal)
    # cv2.waitKey(0)
    print("start", start)
    print("goal", goal)
    # motion_model = holonomic_model.HolonomicModel([10, 10], 1, map, is_discrete=True)
    motion_model = kinematic_bicycle.KinematicBicycle(
        map, 20, 0.1, 4, time_step=1, is_discrete=False
    )
    result = PathPlanningResult()
    planner = RRTStarPlanner(map, motion_model)
    result = planner.plan(start, goal)
    path = []

    if result.path is None or len(result.path) == 0:
        print("!" * 100)
        print("no path found")
        print("^" * 100)
        path = []
    else:
        print(result.path)
        path = result.path

    delta_time = result.timing_data
    expanded = result.expanded_nodes
    print("expanded", len(expanded.keys()))
    # for node in expanded.values():
    #     for n in node.get_ancestry():
    #         assert n in expanded.values(), f"node {n.get_state()} not in expanded nodes"
    viz = viz_plan.VizPlan(map, path, motion_model, start, goal)
    viz.plot_path()
    print("delta time", delta_time)
    print("total time", delta_time["total"])
    # print("expanded", expanded)
    print("number of nodes expanded", len(expanded))
    if not debug:
        return
    # viz.plot_expanded_nodes(expanded)
    # plt.show
    # viz.plot_cost_and_heuristic(expanded)
    smoother = SplineSmoother(motion_model)
    smoothed_paths = [result.path]
    smoothed_paths.extend(smoother.smooth_path(result.path))
    viz = viz_plan.VizPlan(map, smoothed_paths[1], motion_model, start, goal)
    # viz.plot_path()
    # inter_smoother = InterpSmoother(motion_model)
    # inter_path = inter_smoother.smooth_path(result.path)
    # smoothed_paths.extend(inter_path)
    # viz = viz_plan.VizPlan(map, smoothed_paths[2], motion_model, start, goal)
    # viz.plot_path()
    # smoothed_paths.extend(smoother.smooth_path(inter_path[0]))
    # viz = viz_plan.VizPlan(map, smoothed_paths[3], motion_model, start, goal)
    # viz.plot_path()
    bezier_smoother = BezierSmoother(motion_model)
    bezier_path = bezier_smoother.smooth_path(result.path)
    viz = viz_plan.VizPlan(map, bezier_path[0], motion_model, start, goal)
    smoothed_paths.extend(bezier_path)

    time = len(result.path)
    for i, path in enumerate(smoothed_paths):
        control, controld_dt, jerk = motion_model.evaluate_path(path, time)
        fig, axs = plt.subplots(2, 3, figsize=(6.5, 4))
        fig.suptitle(f"Control and Control Derivative for {i}")
        axs[0][0].set_title("Linear Velocity")
        axs[0][0].plot(control[0])
        axs[0][0].axhline(y=motion_model.max_velocity_linear, color="r", linestyle="--")
        axs[0][0].axhline(
            y=-motion_model.max_velocity_linear, color="r", linestyle="--"
        )
        axs[0][1].set_title("Linear Velocity Derivative")
        axs[0][1].plot(controld_dt[0])

        axs[0][2].set_title("Jerk")
        axs[0][2].plot(jerk[0])

        axs[1][0].set_title("Steering Angle")
        axs[1][0].plot(control[1])
        axs[1][0].axhline(
            y=motion_model.max_angular_velocity, color="r", linestyle="--"
        )
        axs[1][0].axhline(
            y=-motion_model.max_angular_velocity, color="r", linestyle="--"
        )
        axs[1][1].set_title("Steering Angle Rate")
        axs[1][1].plot(controld_dt[1])

        axs[1][2].set_title("Steering Jerk")
        axs[1][2].plot(jerk[1])
    plt.show()
    return


def set_start_goal(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(x, y)
        start = (x, y, 0)
        print("start", start)
    if event == cv2.EVENT_RBUTTONDOWN:
        print(x, y)
        goal = (x, y, 0)
        print("goal", goal)
        cv2.destroyAllWindows()
    return


if __name__ == "__main__":
    main(debug=True)
