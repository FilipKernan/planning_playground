import cv2

class Map2d:

    def __init__(self, image_path, grid_size=10):
        self.image_path = image_path
        self.image = cv2.imread(image_path)
        self.height, self.width = self.get_map_dimentions(self.image)
        self.hulls = self.get_map_convex_obstacles(self.image)
        self.map_dimentions, self.hulls = self.get_map_dimentions_and_obstacles(self.image)
        self.grid_size = grid_size
        self.graph = self.create_map_graph(self.image, self.grid_size)


    def import_map(image_path):
        # Load the image using OpenCV
        image = cv2.imread(image_path)

        # Check if the image was loaded successfully
        if image is not None:
            # Perform any further processing on the image here
            # For example, you can apply filters, resize, or perform object detection

            # Display the image
            cv2.imshow('Image', image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print('Failed to load the image.')


    # get the dementions of a map
    def get_map_dimentions(self):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
            # Get the dimensions of the image
        height, width, channels = self.image.shape
        return height, width

    #get the contours of the map as a set of convex polygons
    def get_map_convex_obstacles(self):
        # Load the image using OpenCV

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
        hulls = self.get_map_convex_obstacles(self.image)

        return map_dimentions, hulls

    # create a graph from the map using a grid of nodes and a discretized version of the map
    def create_map_graph(self):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Convert the image to grayscale
        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Apply a threshold to the image
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        # Find contours in the image
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Resize input to "pixelated" size
        pixilized = cv2.resize(input, (self.grid_size, self.grid_size), interpolation=cv2.INTER_LINEAR)

        # Create a grid of nodes
        nodes = {}
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                nodes[i, j] = (pixilized[i, j], (i * self.grid_size, j * self.grid_size))

        # Create a graph from the grid of nodes
        graph = {}
        for node in nodes:
            graph[node] = []
            for neighbor in nodes:
                if neighbor != node:
                    if abs(neighbor[0] - node[0]) <= self.grid_size and abs(neighbor[1] - node[1]) <= self.grid_size:
                        graph[node].append(neighbor)
        self.nodes = nodes
        self.graph = graph
        return graph

    #get the value of the map a a specific point
    def get_map_value(self, point):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the value of the map at a specific point
        value = self.image[point[0], point[1]]
        return value

    # get the balue of the map at a specific point with the grid
    def get_map_value(self, point ):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the value of the map at a specific point with the grid
        value = self.image[point[0], point[1]]
        return value

    def get_neighbors(self, node):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the neighbors of a point
        
        return self.graph[node] 
    
    def get_node(self, point):
        # Load the image using OpenCV

        # Check if the image was loaded successfully
        # Get the node of a point
        
        return self.nodes[point]
    