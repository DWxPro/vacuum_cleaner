import fields2cover as f2c
import cv2 as cv
from matplotlib import pyplot as plt
import math

# robot parameters
robot = f2c.Robot(5, 6)
robot.setMinRadius(2)           # m
robot.linear_curv_change = 0.1  # 1/m^2

# map file
map_file_path = '/home/daniel/ros2_ws/src/PATH_PLANING/maps/Home_Area.pgm'

# settings
print_edges = False
scale = 1

# colors
colors_slam          = {"edge": 0,  "area": 254, "world": 205}
colors_living_room   = {"edge": 0,  "area": 135}
colors_kitchen       = {"edge": 16, "area": 152}
colors_badroom       = {"edge": 32, "area": 169}
colors_corridor      = {"edge": 48, "area": 186}
colors_toilet        = {"edge": 64, "area": 203}
colors_bathroom      = {"edge": 80, "area": 220}
colors_hallway       = {"edge": 96, "area": 237}
colors_workspace     = {"edge": 112,"area": 254}

def edges_to_points(edges):
    points = []
    x_values, y_values = edges.shape
    
    for x in range(0,x_values-1):
        for y in range(0,y_values-1):
            if edges[x][y] == 255:
                points.append([x*scale,y*scale])
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

def print_room(*args):
    f2c.Visualizer.figure(200)
    for room in args:
        f2c.Visualizer.plot(room.cell)
        f2c.Visualizer.plot(room.headlands)
        f2c.Visualizer.plot(room.path)
    f2c.Visualizer.show()

def print_edge(room):
    plt.subplot(121),plt.imshow(image,cmap = 'gray')
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(room.edges,cmap = 'gray')
    plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
    plt.show()

class room:
    def __init__(self, name, color):
        self.name = name
        self.color_edge = color["edge"]
        self.color_area = color["area"]
        self.image = None
        self.edges = None
        self.points = None
        self.cell = None
        self.swath = None
        self.headlands = None
        self.path = None

    def calculate_path(self, image, robot, swath_angle):
        try:
            # remove other colors
            self.image = image.copy()
            self.image[(self.image < 127) & (self.image != self.color_edge)] = colors_slam["world"]
            self.image[(self.image > 127) & (self.image != self.color_area)] = colors_slam["world"]
            self.image[self.image == colors_slam["world"]] = self.color_edge

            # edge detection
            self.edges = cv.Canny(self.image,100,200)

            # generate headlines
            self.points = edges_to_points(self.edges)
            self.ring = points_to_ring(self.points)
            self.cell = f2c.Cells(f2c.Cell(self.ring))

            self.headlands = f2c.HG_Const_gen().generateHeadlands(self.cell, robot.robot_width)

            # generate swaths
            self.swaths = f2c.SG_BruteForce().generateSwaths(swath_angle * math.pi / 180, robot.op_width, self.headlands.getGeometry(0))
            self.swaths = f2c.RP_Boustrophedon().genSortedSwaths(self.swaths)

            # path planning
            reeds_shepp = f2c.PP_ReedsSheppCurves()
            self.path = f2c.PP_PathPlanning().searchBestPath(robot, self.swaths, reeds_shepp)
        except:
            print(self.name + " is not part of the map")

living_room = room("living_room",colors_living_room)
kitchen     = room("kitchen",colors_kitchen)
badroom     = room("badroom",colors_badroom)
corridor    = room("corridor",colors_corridor)
toilet      = room("toilet",colors_toilet)
bathroom    = room("bathroom",colors_bathroom)
hallway     = room("hallway",colors_hallway)
workspace   = room("workspace",colors_workspace)

# read image
image = cv.imread(map_file_path,-1) 

# calculate path
living_room.calculate_path(image, robot, 20)
kitchen.calculate_path(image, robot, 20)
badroom.calculate_path(image, robot, 20)
corridor.calculate_path(image, robot, 20)
toilet.calculate_path(image, robot, 20)
bathroom.calculate_path(image, robot, 20)
hallway.calculate_path(image, robot, 20)
workspace.calculate_path(image, robot, 20)

print_room(living_room, kitchen, corridor, hallway, workspace)
print_edge(kitchen)