import cv2
import planning_playground.map.import_map as import_map
import planning_playground.motion_models.holonomic_model as holonomic_model
import planning_playground.viz.viz_plan as viz_plan
from planning_playground.planners.rrt_star_planner import RRTStarPlanner
from planning_playground.planners.rrt_planner import RRTPlanner
from planning_playground.planners.a_star_planner import AStarPlanner
from planning_playground.planners.types import PathPlanningResult

# create planner builder class
# should take in a map, motion model, heruistic, and planner type
# should have a plan method that takes in a start and goal state
# should return a PathPlanningResult

# create a motion model builder class
# should take in a map, and a motion model type, and the state space

global start
start = (150, 300, 0)
global goal
goal = (900, 900, 0)


def main(debug=False):
    # Create a map
    print("creating map")
    map = import_map.Map2d("planning_playground/map/map_dense750.png", grid_size=60)
    # map = import_map.Map2d("planning_playground/map/map106.png", grid_size=60)
    print("created map")
    # todo: make a ui for chousing the start and goal
    # cv2.imshow("map", map.map)
    # cv2.setMouseCallback("map", set_start_goal)
    # cv2.waitKey(0)
    print("start", start)
    print("goal", goal)
    motion_model = holonomic_model.HolonomicModel([1, 1], 1, map, is_discrete=False)
    result = PathPlanningResult()
    planner = RRTStarPlanner(map, motion_model)
    result = planner.plan(start, goal)
    print(result.path)
    path = result.path
    delta_time = result.timing_data
    expanded = result.expended_nodes
    viz = viz_plan.VizPlan(map, path, motion_model, start, goal)
    viz.plot_path()
    print("delta time", delta_time)
    print("total time", delta_time["total"])
    # print("expanded", expanded)
    print("number of nodes expanded", len(expanded))
    if not debug:
        return
    viz.plot_expanded_nodes(expanded)
    # viz.plot_cost_and_heuristic(expanded)
    # longest_edge = 0
    # print(expanded)
    # for node in expanded.values():
    #     if node is None or node.parent is None:
    #         continue
    #     dist = motion_model.calc_cost(
    #         node.parent.get_state(), node.get_state(), delta_time
    #     )
    #     print(dist)
    #     if dist > longest_edge:
    #         longest_edge = dist

    # print(f"longest edge was {longest_edge}")

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
