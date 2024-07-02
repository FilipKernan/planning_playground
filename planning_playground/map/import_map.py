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
        self.map = self.import_map(image_path)
        self.map_dimensions, self.convex_obstacles = (
            self.get_map_dimensions_and_obstacles()
        )
        self.grid_size = grid_size
        self.create_map_graph()

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

        kernel = np.ones((10, 10), np.uint8)  # this should be a parameter
        dialated = cv2.dilate(thresh, kernel, iterations=2)
        # Find contours in the image
        contours, _ = cv2.findContours(dialated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(self.map, contours, -1, (0, 255, 0), 4)
        # cv2.imshow("hello", self.map)

        hull_list = []
        # for contour in contours:
        #     # Convert the contour to a format suitable for the triangle library
        #     contour_points = contour.squeeze().tolist()
        #     segments = [
        #         [i, (i + 1) % len(contour_points)] for i in range(len(contour_points))
        #     ]

        #     # Create the input dictionary for the triangle library
        #     contour_dict = {"vertices": contour_points, "segments": segments}

        #     # Triangulate the contour using the triangle library
        #     t = triangle.triangulate(contour_dict, "pq0")

        #     # Reconstruct the convex polygons from the triangulation
        #     for triangle_indices in t["triangles"]:
        #         pts = [t["vertices"][i] for i in triangle_indices]
        #         hull_list.append(np.array(pts, dtype=np.int32))
        for contour in contours:
            hull_list.append(contour)
        cv2.drawContours(self.map, hull_list, -1, (0, 127, 255), 1)
        # cv2.imshow("Image", self.map)
        # cv2.waitKey(0)
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

    # create a graph from the map using a grid of nodes and a discretized version of the map
    def create_map_graph(self):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Convert the image to grayscale
        ##  print("creating map graph")
        gray = cv2.cvtColor(self.map, cv2.COLOR_BGR2GRAY)

        # Apply a threshold to the image
        _, thresh = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY_INV)
        # print("thresh", thresh)
        # cv2.imshow("thresh", thresh)

        # Find contours in the image
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        kernel = np.ones((10, 10), np.uint8)  # this should be a parameter
        dilated = cv2.dilate(thresh, kernel, iterations=2)
        # cv2.imshow("dilated", dilated)
        self.collision_map = dilated
        # Resize input to "pixelated" size
        pixilated = cv2.resize(
            dilated, (self.grid_size, self.grid_size), interpolation=cv2.INTER_LINEAR
        )
        self.pixilized = pixilated
        # cv2.imshow("pixilated", pixilated)
        # cv2.waitKey(0)
        cv2.destroyAllWindows()
        print("creating nodes " + str(self.grid_size**2))
        # Create a grid of nodes
        nodes = {}
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                nodes[i, j] = (
                    pixilated[i, j],
                    (i * self.grid_size, j * self.grid_size),
                )

    # get the value of the map a a specific point
    def get_map_collision_value(self, point):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the value of the map at a specific point
        value = self.collision_map[point[1], point[0]]
        return value

    # get the balue of the map at a specific point with the grid
    def get_map_discretized_collision_value(self, point):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the value of the map at a specific point with the grid
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
