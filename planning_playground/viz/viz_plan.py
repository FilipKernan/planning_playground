import cv2
import matplotlib.pyplot as plt
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
            cv2.line(image_copy, point, next_point, (0, 0, 255), 5)

        image_copy = cv2.circle(
            image_copy, self.motion_model.get_points(self.start), 5, (0, 255, 0), -1
        )
        image_copy = cv2.circle(
            image_copy, self.motion_model.get_points(self.goal), 5, (255, 0, 0), -1
        )
        plt.imshow(image_copy)
        plt.title("Path")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.show()

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

    def plot_expanded_nodes(self, expanded):
        # Plot the expanded.values() and connections
        plt.figure(figsize=(10, 8))

        # Plot each node
        for node in expanded.values():
            x, y = node.state[0], node.state[1]
            plt.scatter(x, y, c="skyblue", s=100)  # Plot the node
            # plt.text(x, y, f'({x}, {y})', fontsize=9, ha='right')  # Label the node

        # Plot the connections
        for node in expanded.values():
            for child in node.children:
                x_values = [node.state[0], child.state[0]]
                y_values = [node.state[1], child.state[1]]
                plt.plot(x_values, y_values, "gray")

        # Customize the plot
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("Graph Visualization Based on Node Positions")
        plt.grid(True)
        plt.show()
