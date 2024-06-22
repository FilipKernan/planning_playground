from calendar import c
import cv2
import numpy as np

class Map2d:
    # This only supports square maps for now
    def __init__(self, image_path, grid_size=10):
        self.hulls = None
        self.map_dimentions = None
        self.image = None
        self.image_path = image_path
        self.image = self.import_map(image_path)
        self.map_dimentions, self.hulls = self.get_map_dimentions_and_obstacles()
        self.grid_size = grid_size
        self.create_map_graph()


    def import_map(self, image_path):
        # Load the image using OpenCV
        image = cv2.imread(image_path)
        # Check if the image was loaded successfully
        if image is not None:
            print("Image loaded successfully")
        else:
            print('Failed to load the image.')
        return image


    # get the dementions of a map
    def get_map_dimentions(self):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
            # Get the dimensions of the image
        return self.map_dimentions 

    #get the contours of the map as a set of convex polygons
    def get_map_convex_obstacles(self):
        # Load the image using OpenCV
        # print("getting map convex obstacles")        
        # Check if the image was loaded successfully
        # Convert the image to grayscale
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Apply a threshold to the image
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        # Find contours in the image
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the convex hull object for each contour
        hull_list = []
        for i in range(len(contours)):
            hull = cv2.convexHull(contours[i])
            hull_list.append(hull)

        return hull_list

    # write a function that gets the dimentions of the map and the set of all convex polygons
    def get_map_dimentions_and_obstacles(self):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the dimensions of the image
        height, width, channels = self.image.shape
        map_dimentions = (height, width)    
        hulls = self.get_map_convex_obstacles()

        return map_dimentions, hulls

    # create a graph from the map using a grid of nodes and a discretized version of the map
    def create_map_graph(self):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Convert the image to grayscale
        ##  print("creating map graph")
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Apply a threshold to the image
        _, thresh = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY_INV)
        # print("thresh", thresh)
        cv2.imshow("thresh", thresh)

        # Find contours in the image
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        kernel = np.ones((10, 10), np.uint8) # this should be a parameter
        dialated = cv2.dilate(thresh, kernel, iterations=2)
        cv2.imshow("dialated", dialated)
        self.collision_map = dialated
        # Resize input to "pixelated" size
        pixilized = cv2.resize(dialated, (self.grid_size, self.grid_size), interpolation=cv2.INTER_LINEAR)
        self.pixilized = pixilized
        cv2.imshow("pixilized", pixilized)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        print("creating nodes " + str(self.grid_size**2))
        # Create a grid of nodes
        nodes = {}
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                nodes[i, j] = (pixilized[i, j], (i * self.grid_size, j * self.grid_size))

    #get the value of the map a a specific point
    def get_map_collision_value(self, point):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the value of the map at a specific point
        value = self.collision_map[point[1], point[0]]
        return value

    # get the balue of the map at a specific point with the grid
    def get_map_discretized_collision_value(self, point ):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the value of the map at a specific point with the grid
        value = self.pixilized[ point[1], point[0]]
        # print("value", value)
        return value
    
    def get_map_point_in_collision(self, point, descritized_map = True):
        if descritized_map:
            print("checking point:", point)
            point = (point[0] // (self.map_dimentions[0] // self.grid_size), point[1] // (self.map_dimentions[1] // self.grid_size))
            print("checking against disc:", point)
            return self.get_map_discretized_collision_value(point) != 0
        else:
            # convert the point to the discritized map
            return self.get_map_collision_value(point) != 0

    def get_node(self, point):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the node of a point
        
        return self.nodes[point]
    