import cv2
from matplotlib import pyplot as plt

import planning_playground.map.import_map as import_map
import planning_playground.motion_models.holonomic_model as holonomic_model
import planning_playground.motion_models.kinematic_unicycle as kinematic_unicycle
import planning_playground.motion_models.kinematic_bicycle as kinematic_bicycle
import planning_playground.viz.viz_plan as viz_plan
from planning_playground.planners.a_star_planner import AStarPlanner
from planning_playground.planners.rrt_planner import RRTPlanner
from planning_playground.planners.rrt_star_planner import RRTStarPlanner
from planning_playground.planners.types import PathPlanningResult

# create planner builder class
# should take in a map, motion model, heruistic, and planner type
# should have a plan method that takes in a start and goal state
# should return a PathPlanningResult

# create a motion model builder class
# should take in a map, and a motion model type, and the state space

global start
start = (150, 300, -0.5)
global goal
goal = (900, 900, 0.0)


def main(debug=False):
    # Create a map
    print("creating map")
    # map = import_map.Map2d("planning_playground/map/map_dense750.png", grid_size=60)
    map = import_map.Map2d("planning_playground/map/map_empty.png", grid_size=100)
    print("created map")
    # todo: make a ui for chousing the start and goal
    # cv2.imshow("map", map.map)
    # cv2.setMouseCallback("map", set_start_goal)
    # cv2.waitKey(0)
    print("start", start)
    print("goal", goal)
    # motion_model = holonomic_model.HolonomicModel([1, 1], 1, map, is_discrete=False)
    motion_model = kinematic_bicycle.KinematicBicycle(
        map, 20, 0.01, 2, time_step=1, is_discrete=True
    )
    result = PathPlanningResult()
    planner = AStarPlanner(map, motion_model)
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
    sorted_expanded = sorted(expanded.values(), key=lambda x: x.get_heuristic())
    with open("expanded_nodes.txt", "w") as f:
        for node in sorted_expanded:
            f.write(f"{node.get_state()} \n")
    viz.plot_expanded_nodes(expanded)
    plt.show()
    viz.plot_cost_and_heuristic(expanded)

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
