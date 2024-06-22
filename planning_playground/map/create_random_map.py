import cv2
from matplotlib import contour
import numpy as np

class MapCreator:
    def __init__(self, width, height, grid_size, min_obstacles=15, max_obstacles=50):
        self.width = width
        self.height = height
        self.grid_size = grid_size
        self.min_obstacles = min_obstacles
        self.max_obstacles = max_obstacles

    def create_map(self):
        # Create a blank image
        
        self.map = self.create_random_map()

        # # Save the image
        image_name = "map_dense" + str(np.random.randint(0, 1000)) + ".png"
        cv2.imwrite(image_name, self.map)

        return self.map 


    def create_random_map(self):
        # Create a blank image
        image = np.zeros((self.width, self.height, 3), np.uint8)
        image.fill(255)
        print(type(image))
        image = np.ascontiguousarray(image, dtype=np.uint8)
        print(type(image))
        number_of_obstacles = np.random.randint(self.min_obstacles, self.max_obstacles)
        rectangle_size = min(self.width, self.height) // 10
        # Draw random rectangles
        original_points = []
        for _ in range(number_of_obstacles):
            center_point = (np.random.randint(0, self.width), np.random.randint(0, self.height))
            x1 = max(0.0, center_point[0] - rectangle_size // 2)
            y1 = max( 0.0, center_point[1] - rectangle_size // 2)
            x2 = min(self.width, center_point[0] + rectangle_size // 2)
            y2 = min(self.height, center_point[1] + rectangle_size // 2)
            points = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]])
            original_points.append(points)
            rotated_points = self.rotate(center_point, points, np.random.randint(0, 360))
            cv2.drawContours(image, [rotated_points], -1, (0, 0, 0), -1)
        
        return image

    def rotate(self, center_point, points, angle):
        ANGLE = np.deg2rad(angle)
        SIN = np.sin(ANGLE)
        COS = np.cos(ANGLE)
        
        c_x, c_y = np.mean(points, axis=0)

        return np.array(
            [
                [
                    c_x + COS * (px - c_x) - SIN * (py - c_y),
                    c_y + SIN * (px - c_x) + COS * (py - c_y),
                ]
                for px, py in points
            ]
        ).astype(int)
    

if __name__ == '__main__':
    map_creator = MapCreator(1000, 1000, 10)
    cv2.imshow("map", map_creator.create_map())
    print("Map created successfully")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    exit()
