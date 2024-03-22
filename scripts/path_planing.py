import fields2cover as f2c
import cv2 as cv
from matplotlib import pyplot as plt
import math
import yaml

# slam color
colors_slam          = {"edge": 0,  "area": 254, "world": 205}

def edges_to_points(edges, map):
    points = []
    x_values, y_values = edges.shape
    scale = map.scale
    origin_x = map.origin[0]
    orging_y = map.origin[1]

    for x in range(0,x_values-1):
        for y in range(0,y_values-1):
            if edges[x][y] == 255:
                points.append([x*scale+origin_x,y*scale+orging_y])
    return points

def points_to_ring(points):
    ring = f2c.LinearRing()

    start_point = points[0]
    finish_point = start_point
    while len(points) > 1:
        points.remove(start_point)
        
        distances = []
        for point in points:
            dx = point[0] - start_point[0]
            dy = point[1] - start_point[1]
            distances.append(dx**2+dy**2)
            
        next_point = points[distances.index(min(distances))]
        ring.addGeometry(f2c.Point(start_point[0],start_point[1]))
        start_point = next_point

    ring.addGeometry(f2c.Point(finish_point[0],finish_point[1]))
    return ring

def print_room_path(*args):
    f2c.Visualizer.figure(200)
    for room in args:
        f2c.Visualizer.plot(room.cell)
        f2c.Visualizer.plot(room.headlands)
        f2c.Visualizer.plot(room.path)
    f2c.Visualizer.show()

def print_image(image):
    plt.imshow(image,cmap = 'gray')
    plt.axis('off')
    plt.show()

def get_points_from_path(path):
    points = []
    serial_path = f2c.Path.serializePath(path).split('\n')[0:-1]
    
    for raw in serial_path:
        values = raw.split(" ")
        x = values[0]
        y = values[1]
        points.append([x,y])
    
    return points

class Map:
    def __init__(self, image, yaml_file):
        self.image = cv.rotate(cv.imread(image,-1),cv.ROTATE_90_CLOCKWISE)
        with open(yaml_file, 'r') as file:
            self.data = yaml.safe_load(file)
            self.scale = self.data["resolution"]          #[m/pixel]
            self.origin = self.data["origin"]             #[x,y,z]      

class Room:
    def __init__(self, name, map, color):
        self.name = name
        self.map = map
        self.color_edge = color["edge"]
        self.color_area = color["area"]
        self.image = None
        self.edges = None
        self.points = None
        self.cell = None
        self.swath = None
        self.headlands = None
        self.path = None

    def calculate_path(self, robot, swath_angle, start_point):
        """swath_angle [°] start_point[1-4]"""
        try:
            # remove other colors
            self.image = self.map.image.copy()
            self.image[(self.image < 127) & (self.image != self.color_edge)] = colors_slam["world"]
            self.image[(self.image > 127) & (self.image != self.color_area)] = colors_slam["world"]
            self.image[self.image == colors_slam["world"]] = self.color_edge

            # edge detection
            self.edges = cv.Canny(self.image,100,200)

            # generate headlines
            self.points = edges_to_points(self.edges, self.map)
            self.ring = points_to_ring(self.points)
            self.cell = f2c.Cells(f2c.Cell(self.ring))

            self.headlands = f2c.HG_Const_gen().generateHeadlands(self.cell, robot.f2c_robot.robot_width)

            # generate swaths
            self.swaths = f2c.SG_BruteForce().generateSwaths(swath_angle * math.pi / 180, robot.f2c_robot.op_width, self.headlands.getGeometry(0))
            self.swaths = f2c.RP_Boustrophedon().genSortedSwaths(self.swaths, start_point)

            # path planning
            reeds_shepp = f2c.PP_ReedsSheppCurves()
            self.path = f2c.PP_PathPlanning().searchBestPath(robot.f2c_robot, self.swaths, reeds_shepp)
            
        except:
            print(self.name + " is not part of the map")
    
    def print_edge(self):
        plt.subplot(121),plt.imshow(self.map.image,cmap = 'gray')
        plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(self.edges,cmap = 'gray')
        plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
        plt.show()

class Robot():
    def __init__(self, robot_width, operation_width):
        """robot_width [m] operation_width [m]"""
        self.f2c_robot = f2c.Robot()
        self.f2c_robot.robot_width = robot_width   # [m]
        self.f2c_robot.op_width = operation_width  # [m]
        self.f2c_robot.setMinRadius(0)             # [m]

def main():
    # map file
    map_image_path = 'vacuum_cleaner/maps/Home_Area.pgm'
    map_yaml_path = 'vacuum_cleaner/maps/Home1.yaml'

    # colors
    colors_living_room   = {"edge": 0,  "area": 135}
    colors_kitchen       = {"edge": 16, "area": 152}
    colors_badroom       = {"edge": 32, "area": 169}
    colors_corridor      = {"edge": 48, "area": 186}
    colors_toilet        = {"edge": 64, "area": 203}
    colors_bathroom      = {"edge": 80, "area": 220}
    colors_hallway       = {"edge": 96, "area": 237}
    colors_workspace     = {"edge": 112,"area": 254}

    # robot setting
    robot_width = 0.150         # [m]
    operation_width = 0.150     # [m]

    # init map
    home_map = Map(map_image_path, map_yaml_path)

    # init rooms
    living_room = Room("living_room", home_map, colors_living_room)
    kitchen     = Room("kitchen", home_map, colors_kitchen)
    badroom     = Room("badroom", home_map, colors_badroom)
    corridor    = Room("corridor", home_map, colors_corridor)
    toilet      = Room("toilet", home_map, colors_toilet)
    bathroom    = Room("bathroom", home_map, colors_bathroom)
    hallway     = Room("hallway", home_map, colors_hallway)
    workspace   = Room("workspace", home_map, colors_workspace)

    # init robot
    vacuum_cleaner = Robot(robot_width, operation_width)

    # calculate path
    living_room.calculate_path(vacuum_cleaner, 110, 1)
    kitchen.calculate_path(vacuum_cleaner, 110, 2)
    badroom.calculate_path(vacuum_cleaner, 110, 3)
    corridor.calculate_path(vacuum_cleaner, 110,4)
    toilet.calculate_path(vacuum_cleaner, 110, 1)
    bathroom.calculate_path(vacuum_cleaner, 110, 2)
    hallway.calculate_path(vacuum_cleaner, 110, 3)
    workspace.calculate_path(vacuum_cleaner, 110, 4)

    # print result
    print_room_path(kitchen, living_room, kitchen, corridor, hallway, workspace)
    print(get_points_from_path(kitchen.path))

if __name__ == '__main__':
    main()