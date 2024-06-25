import planning_playground.map.import_map as import_map
import planning_playground.motion_models.holonomic_model as holonomic_model
import planning_playground.planners.rrt_planner as rrt_planner
import planning_playground.planners.a_star_planner as astar_planner
from planning_playground.planners.types import PathPlanningResult
from planning_playground.planners.types import Node
import planning_playground.viz.viz_plan as viz_plan
import networkx as nx
import cv2
import matplotlib.pyplot as plt
import numpy as np


def main(debug=False):
    # Create a map
    print("creating map")
    map = import_map.Map2d("planning_playground/map/map_dense750.png", grid_size=60)
    print("created map")
    # todo: make a ui for chousing the start and goal
    start = (0, 0, 0)
    goal = (900, 900, 0)
    motion_model = holonomic_model.HolonomicModel([1, 1], 1, map, is_discrete=True)
    result = PathPlanningResult()
    a_star = astar_planner.AStarPlanner(map,  motion_model)
    result = a_star.plan(start, goal)
    print(result.path)
    path = result.path
    delta_time = result.timing_data
    expanded = result.expended_nodes
    viz = viz_plan.VizPlan(map, path, motion_model, start, goal)
    viz.plot_path()
    print("delta time", delta_time)
    sum_val = sum(value for key, value in delta_time.items() if key != "total" and key != "expanding")
    print("total time", delta_time["total"] )
    # print("expanded", expanded)
    print("number of nodes expanded", len(expanded))
    if debug == False:
        return
    return


if __name__ == '__main__':
    main(debug=True)
