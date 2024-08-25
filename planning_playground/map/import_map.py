import cv2
import numpy as np
import triangle

import planning_playground.map.abstract_map as abstract_map


class Map2d(abstract_map.AbstractMap):
    # This only supports square maps for now
    def __init__(self, image_path, grid_size=10):
        self.convex_obstacles = None
        self.map_dimensions = None
        self.map = None
        self.image_path = image_path
        self.grid_size = grid_size
        self.map = self.import_map(image_path)
        self.map_dimensions, self.convex_obstacles = (
            self.get_map_dimensions_and_obstacles()
        )

    def import_map(self, image_path):
        # Load the image using OpenCV
        image = cv2.imread(image_path)
        # Check if the image was loaded successfully
        if image is not None:
            print("Image loaded successfully")
        else:
            print("Failed to load the image.")
        return image

    # get the dementions of a map
    def get_map_dimensions(self):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the dimensions of the image
        return self.map_dimensions

    # get the contours of the map as a set of convex polygons
    def get_map_convex_obstacles(self):
        # Load the image using OpenCV
        # print("getting map convex obstacles")
        # Check if the image was loaded successfully
        # Convert the image to grayscale
        gray = cv2.cvtColor(self.map, cv2.COLOR_BGR2GRAY)
        gray = cv2.rectangle(
            gray, (0, 0), (gray.shape[1], gray.shape[0]), (255, 255, 255), 5
        )
        # cv2.imshow("gray", gray)
        # Apply a threshold to the image
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((5, 5), np.uint8)  # this should be a parameter
        dilated = cv2.dilate(thresh, kernel, iterations=5)
        # Find contours in the image
        contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(self.map, contours, -1, (0, 255, 0), 4)
        # cv2.imshow("hello", self.map)

        hull_list = []
        for contour in contours:
            hull_list.append(contour)
        cv2.drawContours(self.map, hull_list, -1, (0, 127, 255), 1)
        # cv2.imshow("Image", self.map)
        # cv2.waitKey(0)
        self.collision_map = dilated
        # Resize input to "pixelated" size
        pixilated = cv2.resize(
            dilated, (self.grid_size, self.grid_size), interpolation=cv2.INTER_LINEAR
        )
        self.pixilized = pixilated
        cv2.destroyAllWindows()
        return hull_list

    # write a function that gets the dimensions of the map and the set of all convex polygons
    def get_map_dimensions_and_obstacles(self):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the dimensions of the image
        height, width, channels = self.map.shape
        map_dimensions = (height, width)
        hulls = self.get_map_convex_obstacles()

        return map_dimensions, hulls

    # get the value of the map a a specific point
    def get_map_collision_value(self, point):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the value of the map at a specific point
        point = (int(point[0]), int(point[1]))
        if point[0] < 0 or point[1] < 0:
            return 1
        if point[0] >= self.map_dimensions[0] or point[1] >= self.map_dimensions[1]:
            return 1
        value = self.collision_map[int(point[1]), int(point[0])]
        return value

    # get the balue of the map at a specific point with the grid
    def get_map_discretized_collision_value(self, point):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the value of the map at a specific point with the grid
        point = (int(point[0]), int(point[1]))
        if point[0] < 0 or point[1] < 0:
            return 1
        if point[0] >= self.pixilized.shape[0] or point[1] >= self.pixilized.shape[1]:
            return 1
        value = self.pixilized[point[1], point[0]]
        # print("value", value)
        return value

    def scale(self, OldValue):
        OldRange = 0 - self.map_dimensions[0]
        NewRange = 0 - self.grid_size
        NewValue = (((OldValue - 0) * NewRange) / OldRange) + 0
        return NewValue

    def get_map_point_in_collision(self, point, descritized_map=True):
        if descritized_map:
            # print("checking point:", point)
            point = (int(self.scale(point[0])), int(self.scale(point[1])))
            # print("checking against disc:", point)
            return self.get_map_discretized_collision_value(point) != 0
        else:
            # convert the point to the discritized map
            return self.get_map_collision_value(point) != 0

    def get_node(self, point):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the node of a point

        return self.nodes[point]
