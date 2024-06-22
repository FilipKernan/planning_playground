import planning_playground.map.import_map as import_map
import planning_playground.motion_models.holonomic_model as holonomic_model
import planning_playground.planners.a_star_planner as a_star_planner
import planning_playground.viz.viz_plan as viz
import cv2
import matplotlib.pyplot as plt
import numpy as np


if __name__ == '__main__':
    # Create a map
    print("creating map")
    map = import_map.Map2d("planning_playground/map/map991.png", 100)
    print("created map")
    # todo: make a ui for chousing the start and goal
    start = (0, 0, 0)
    goal = (500, 500, 0)
    motion_model = holonomic_model.HolonomicModel([1, 1], 1, map)
    result = a_star_planner.PathPlanningResult([], {}, {})
    a_star = a_star_planner.AStarPlanner(map, motion_model)
    result = a_star.plan(start, goal)
    print(result.path)
    path = result.path
    delta_time = result.timing_data
    expanded = result.expended_nodes
    viz = viz.VizPlan(map, path, motion_model, start, goal)
    viz.plot_map()
    viz.plot_path()
    print("delta time", delta_time)
    sum_val = sum(value for key, value in delta_time.items() if key != "total" and key != "expanding")
    print(sum_val)
    # print("expanded", expanded)