import cv2
import matplotlib.pyplot as plt
import matplotlib.quiver as quiver
from planning_playground.planners.types import Node

import numpy as np


class VizPlan:
    def __init__(self, map, path: list[tuple[float]], motion_model, start, goal):
        self.map = map
        self.path = path
        self.motion_model = motion_model
        self.start = start
        self.goal = goal

    def plot_map(self):
        image_copy = self.map.map.copy()
        # make the image copy 3 channels
        # resize the image
        plt.imshow(image_copy)

    def linear_interpolate(
        self, point: float, old_range: tuple[float], new_range: tuple[float]
    ):
        _, old_max = old_range
        _, new_max = new_range
        old_value = point
        new_value = max(0, new_max * (old_value / old_max))
        return int(new_value)

    def plot_path(self):
        image_copy = self.map.map.copy()
        # make the image copy 3 channels
        for i in range(len(self.path) - 1):
            point = self.motion_model.get_points(self.path[i])
            next_point = self.motion_model.get_points(self.path[i + 1])
            print("point", point)
            print("next_point", next_point)
            cv2.line(image_copy, point, next_point, (0, 0, 255), 5)

        image_copy = cv2.circle(
            image_copy, self.motion_model.get_points(self.start), 5, (0, 255, 0), -1
        )
        image_copy = cv2.circle(
            image_copy, self.motion_model.get_points(self.goal), 5, (255, 0, 0), -1
        )
        plt.figure()
        plt.imshow(image_copy)
        plt.title("Path")
        plt.xlabel("X")
        plt.ylabel("Y")

    def get_plotted_map(self):
        image_copy = self.map.map.copy()
        plt.figure(figsize=(10, 8))
        start = self.start[0], self.start[1]
        goal = self.goal[0], self.goal[1]
        image_copy = cv2.circle(image_copy, start, 5, (0, 255, 0), -1)
        image_copy = cv2.circle(image_copy, goal, 5, (255, 0, 0), -1)
        plt.imshow(image_copy, cmap="gray")
        plt.title("Map Visualization")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.grid(True)

    def plot_map(self):
        # Plot the map
        self.get_plotted_map()
        plt.show()

    def plot_cost_and_heuristic(self, expanded):
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
        sc1 = axs[0].scatter(x, y, c=costs, cmap="viridis", marker="o")
        axs[0].set_title("Cost")
        axs[0].set_xlabel("X")
        axs[0].set_ylabel("Y")
        fig.colorbar(sc1, ax=axs[0], label="Cost")

        # Plot heuristics
        sc2 = axs[1].scatter(x, y, c=heuristics, cmap="viridis", marker="o")
        axs[1].set_title("Heuristic")
        axs[1].set_xlabel("X")
        axs[1].set_ylabel("Y")
        fig.colorbar(sc2, ax=axs[1], label="Heuristic")

        # Plot total costs
        sc3 = axs[2].scatter(x, y, c=total_costs, cmap="viridis", marker="o")
        axs[2].set_title("Total Cost")
        axs[2].set_xlabel("X")
        axs[2].set_ylabel("Y")
        fig.colorbar(sc3, ax=axs[2], label="Total Cost")

        # Display the plots
        plt.tight_layout()
        plt.show()

    def plot_expanded_nodes(self, expanded: dict[tuple[float], Node]):
        # Plot the expanded.values() and connections
        self.get_plotted_map()

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        # get the max cost in the dict
        xs = []
        ys = []
        thetas = []
        costs = []
        # Plot each node
        for node in expanded.values():
            if node.parent is None:
                continue
            x, y, z = node.get_state()[0], node.get_state()[1], node.get_state()[2]
            # convert the total cost into a color for matplotlib
            xs.append(x)
            ys.append(y)
            thetas.append(z)
            costs.append(node.get_total_cost())
            parent_x = node.parent.get_state()[0]
            parent_y = node.parent.get_state()[1]
            parent_z = node.parent.get_state()[2]
            plt.plot([x, parent_x], [y, parent_y], [z, parent_z], "gray")
        # plt.text(x, y, f'({x}, {y})', fontsize=9, ha='right')  # Label the node

        sc = ax.scatter(
            xs,
            ys,
            zs=thetas,
            c=costs,
            cmap="viridis",
            s=100,
        )  # Plot the node
        fig.colorbar(sc, ax=ax, label="Cost")
        # Plot the connections
        plt.show()
        states = [node.state for node in expanded.values()]
        for node in expanded.values():
            # for child in node.children:
            #     x_values = [node.state[0], child.state[0]]
            #     y_values = [node.state[1], child.state[1]]
            #     plt.plot(x_values, y_values, "gray")
            # if node.parent is None and node.state != (150, 300, 0):
            # print("no parent")
            # print("node", node.state)
            # print("parent", node.parent)
            # print("node children", node.children)

            # if (
            #     node.state != (150, 300, 0)
            #     and node.parent.state != (150, 300, 0)
            #     and node.parent.parent is not None
            #     and node.parent.state != (150, 300, 0)
            # ):
            #     if node.parent.parent.get_state() not in states:
            #         print("no parent")
            #         print("node", node.state)
            #         print("parent", node.parent)
            #         print("node children", node.children)
            # exit(1)

            if node.state != self.start:
                parent_x_values = [node.state[0], node.parent.state[0]]
                parent_y_values = [node.state[1], node.parent.state[1]]
                parent_x = node.parent.state[0]
                parent_y = node.parent.state[1]
                current_x = node.state[0]
                current_y = node.state[1]
                plt.arrow(
                    parent_x,
                    parent_y,
                    current_x - parent_x,
                    current_y - parent_y,
                    head_width=5.0,
                    head_length=5,
                    fc="red",
                    ec="red",
                )

        # Customize the plot
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Graph Visualization Based on Node Positions")
        plt.grid(True)
