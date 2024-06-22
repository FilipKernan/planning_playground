import cv2
import numpy as np
import matplotlib.pyplot as plt
import planning_playground.planners.a_star_planner as a_star_planner


class VizPlan:
    def __init__(self, map, path: list[tuple[float]], motion_model, start, goal):
        self.map = map
        self.path = path
        self.motion_model = motion_model
        self.start = start
        self.goal = goal

    def plot_map(self):
        image_copy = self.map.image
        # make the image copy 3 channels
        start = self.start[0], self.start[1]
        goal = self.goal[0], self.goal[1]
        image_copy = cv2.circle(image_copy, start, 5, (0, 255, 0), -1)
        image_copy = cv2.circle(image_copy, goal, 5, (255, 0, 0), -1)
        # resize the image
        cv2.imshow("map", image_copy)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def linear_interpolate(self, point: float, old_range: tuple[float], new_range: tuple[float]):
        old_min, old_max = old_range
        new_min, new_max = new_range
        old_value = point
        new_value = max(0, new_max * (old_value / old_max))
        return int(new_value)

    def plot_path(self): 
        image_copy = self.map.image
        # make the image copy 3 channels
        old_range = (0, self.map.grid_size)
        new_range = (0, self.map.image.shape[0])
        for i in range(len(self.path) - 1):
            point = self.motion_model.get_points(self.path[i])
            next_point = self.motion_model.get_points(self.path[i + 1])
            cv2.line(image_copy, point, next_point, (0, 0, 255), 5)
 
        image_copy = cv2.circle(image_copy, self.motion_model.get_points(self.start), 5, (0, 255, 0), -1)
        image_copy = cv2.circle(image_copy, self.motion_model.get_points(self.goal), 5, (255, 0, 0), -1)
        cv2.imshow("path", image_copy)
        cv2.waitKey(0)
        cv2.destroyAllWindows()