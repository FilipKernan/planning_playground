import planning_playground.map.import_map as import_map
import planning_playground.motion_models.holonomic_model as holonomic_model
import planning_playground.planners.rrt_planner as rrt_planner
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
    motion_model = holonomic_model.HolonomicModel([1, 1], 1, map, is_discrete=False)
    result = PathPlanningResult()
    a_star = rrt_planner.RRTPlanner(map, motion_model)
    result = a_star.plan(start, goal)
    print(result.path)
    path = result.path
    delta_time = result.timing_data
    expanded = result.expended_nodes
    viz = viz_plan.VizPlan(map, path, motion_model, start, goal)
    viz.plot_map()
    viz.plot_path()
    print("delta time", delta_time)
    sum_val = sum(value for key, value in delta_time.items() if key != "total" and key != "expanding")
    print("total time", delta_time["total"] )
    # print("expanded", expanded)
    print("number of nodes expanded", len(expanded))
    if debug == False:
        return
    # Extract data
    x = []
    y = []
    costs = []
    heuristics = []
    total_costs = []

    for (x_coord, y_coord, _), node in expanded.items():
        x.append(x_coord)
        y.append(y_coord)
        costs.append(node.get_cost())
        heuristics.append(node.get_heuristic())
        total_costs.append(node.get_total_cost())

    # Convert lists to numpy arrays
    x = np.array(x)
    y = np.array(y)
    costs = np.array(costs)
    heuristics = np.array(heuristics)
    total_costs = np.array(total_costs)

    # Create subplots
    fig, axs = plt.subplots(1, 3, figsize=(18, 6))

    # Plot costs
    sc1 = axs[0].scatter(x, y, c=costs, cmap='viridis', marker='o')
    axs[0].set_title('Cost')
    axs[0].set_xlabel('X')
    axs[0].set_ylabel('Y')
    fig.colorbar(sc1, ax=axs[0], label='Cost')

    # Plot heuristics
    sc2 = axs[1].scatter(x, y, c=heuristics, cmap='viridis', marker='o')
    axs[1].set_title('Heuristic')
    axs[1].set_xlabel('X')
    axs[1].set_ylabel('Y')
    fig.colorbar(sc2, ax=axs[1], label='Heuristic')

    # Plot total costs
    sc3 = axs[2].scatter(x, y, c=total_costs, cmap='viridis', marker='o')
    axs[2].set_title('Total Cost')
    axs[2].set_xlabel('X')
    axs[2].set_ylabel('Y')
    fig.colorbar(sc3, ax=axs[2], label='Total Cost')

    # Display the plots
    plt.tight_layout()
    plt.show()

    # Plot the expanded.values() and connections
    plt.figure(figsize=(10, 8))

    # Plot each node
    for node in expanded.values():
        x, y = node.state[0], node.state[1]
        plt.scatter(x, y, c='skyblue', s=100)  # Plot the node
        # plt.text(x, y, f'({x}, {y})', fontsize=9, ha='right')  # Label the node

    # Plot the connections
    for node in expanded.values():
        for child in node.children:
            x_values = [node.state[0], child.state[0]]
            y_values = [node.state[1], child.state[1]]
            plt.plot(x_values, y_values, 'gray')

    # Customize the plot
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Graph Visualization Based on Node Positions')
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    main()
