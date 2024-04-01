import fields2cover as f2c
import cv2 as cv
from matplotlib import pyplot as plt
import math
import yaml

# slam color
colors_slam = {"edge": 0,  "area": 254, "world": 205}

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

def print_f2c_object(object):
    f2c.Visualizer.figure(200)
    f2c.Visualizer.plot(object)
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
        x = float(values[0])
        y = float(values[1])
        points.append([x,y])
    
    cleaned_points = []
    for i in range(1, len(points)-1):
        dx = points[i][0] - points[i-1][0]
        dy = points[i][1] - points[i-1][1]
        distance = math.sqrt(dx**2 + dy**2)

        if distance >= 0.02:
            cleaned_points.append(points[i])
    
    return cleaned_points

def get_poses_from_path(path):
    poses = []
    serial_path = f2c.Path.serializePath(path).split('\n')[0:-1]
    
    for raw in serial_path:
        values = raw.split(" ")
        x = float(values[0])
        y = float(values[1])
        angle = float(values[3])
        poses.append([x,y,angle])
    
    cleaned_poses = []
    for i in range(1, len(poses)-1):
        dx = poses[i][0] - poses[i-1][0]
        dy = poses[i][1] - poses[i-1][1]
        distance = math.sqrt(dx**2 + dy**2)

        if distance >= 0.02:
            cleaned_poses.append(poses[i])

    return cleaned_poses

class Map:
    def __init__(self, image, yaml_file):
        self.image = cv.rotate(cv.imread(image,-1),cv.ROTATE_90_CLOCKWISE)
        with open(yaml_file, 'r') as file:
            self.data = yaml.safe_load(file)
            self.scale = self.data["resolution"]          #[m/pixel]
            self.origin = self.data["origin"]             #[x,y,z]      

class Room:
    def __init__(self, name, map, color, headlands_offset):
        self.name = name
        self.map = map
        self.color_edge = color["edge"]
        self.color_area = color["area"]
        self.headlands_offset = headlands_offset
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

            self.headlands = f2c.HG_Const_gen().generateHeadlands(self.cell, robot.f2c_robot.robot_width + self.headlands_offset)

            # generate swaths
            self.swaths = f2c.SG_BruteForce().generateSwaths(swath_angle * math.pi / 180, robot.f2c_robot.op_width, self.headlands.getGeometry(0))
            self.swaths = f2c.RP_Boustrophedon().genSortedSwaths(self.swaths, start_point)

            # path planning
            #reeds_shepp = f2c.PP_ReedsSheppCurves()
            #self.path = f2c.PP_PathPlanning().searchBestPath(robot.f2c_robot, self.swaths, reeds_shepp)
            
            dubins = f2c.PP_DubinsCurves()
            self.path = f2c.PP_PathPlanning().searchBestPath(robot.f2c_robot, self.swaths, dubins)

        except:
            print(self.name + " is not part of the map")
    
    def print_edge(self):
        plt.subplot(121),plt.imshow(self.map.image,cmap = 'gray')
        plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(self.edges,cmap = 'gray')
        plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
        plt.show()

class Robot:
    def __init__(self, robot_width, operation_width):
        """robot_width [m] operation_width [m]"""
        self.f2c_robot = f2c.Robot()
        self.f2c_robot.robot_width = robot_width   # [m]
        self.f2c_robot.op_width = operation_width  # [m]
        self.f2c_robot.setMinRadius(0.05)             # [m]

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
    headlands_offset = 0.3      #[m]

    living_room = Room("living_room", home_map, colors_living_room, headlands_offset)
    kitchen     = Room("kitchen", home_map, colors_kitchen, headlands_offset)
    badroom     = Room("badroom", home_map, colors_badroom, headlands_offset)
    corridor    = Room("corridor", home_map, colors_corridor, headlands_offset)
    toilet      = Room("toilet", home_map, colors_toilet, headlands_offset)
    bathroom    = Room("bathroom", home_map, colors_bathroom, headlands_offset)
    hallway     = Room("hallway", home_map, colors_hallway, headlands_offset)
    workspace   = Room("workspace", home_map, colors_workspace, headlands_offset)

    # init robot
    vacuum_cleaner = Robot(robot_width, operation_width)

    # calculate path
    living_room.calculate_path(vacuum_cleaner, 100, 1)
    kitchen.calculate_path(vacuum_cleaner, 110, 2)
    badroom.calculate_path(vacuum_cleaner, 110, 3)
    corridor.calculate_path(vacuum_cleaner, 110, 1)
    toilet.calculate_path(vacuum_cleaner, 110, 1)
    bathroom.calculate_path(vacuum_cleaner, 110, 2)
    hallway.calculate_path(vacuum_cleaner, 110, 3)
    workspace.calculate_path(vacuum_cleaner, 20, 4)

    # print result
    #print_room_path(living_room, kitchen, corridor, hallway, workspace)   
    #print(get_points_from_path(workspace.path))

    export(workspace.path)


def export(path):
    # save for testing
    def array_to_txt(array, filename):
        with open(filename, 'w') as file:
            file.write('poses = [')
            for i, value in enumerate(array):
                file.write(str(value))
                if (i + 1) % 5 == 0 and i != len(array) - 1:
                    file.write(', \n')
                else:
                    file.write(', ')
            file.write(']')

    # Beispielaufruf:
    my_array = get_poses_from_path(path)
    array_to_txt(my_array, 'array.txt')


if __name__ == '__main__':
    main()