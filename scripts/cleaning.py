import fields2cover as f2c
import cv2 as cv
from matplotlib import pyplot as plt
import math

# robot parameters
robot = f2c.Robot(5, 6)
robot.setMinRadius(2)  # m
robot.linear_curv_change = 0.1  # 1/m^2

# map file
path = '/home/daniel/ros2_ws/src/PATH_PLANING/maps/Home1.pgm'

# room colors
kitchen = 120
kitchen_edge = 122

# settings
print_edges = True
scale = 1
swath_angle = 20    # [%]

# colors
slam_black = 0
slam_gray = 205
slam_white = 254

# generate line ring out of points
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

# read image
image = cv.imread(path,-1) 
original_image = image.copy()
image[image == slam_gray] = slam_black

# edge detection
edges = cv.Canny(image,100,200)

# generate headlands
points = []
x_values, y_values = edges.shape

for x in range(0,x_values-1):
    for y in range(0,y_values-1):
        if edges[x][y] == 255:
            points.append([x*scale,y*scale])

ring = points_to_ring(points)
cells = f2c.Cells(f2c.Cell(ring))

headlands = f2c.HG_Const_gen().generateHeadlands(cells, robot.robot_width)

# generate swaths
bf = f2c.SG_BruteForce()
swath_length = f2c.OBJ_SwathLength()
swaths = bf.generateSwaths(swath_angle * math.pi / 180, robot.op_width, headlands.getGeometry(0))

swaths = f2c.RP_Boustrophedon().genSortedSwaths(swaths)

# path planning
reeds_shepp = f2c.PP_ReedsSheppCurves()
path = f2c.PP_PathPlanning().searchBestPath(robot, swaths, reeds_shepp)

# print output
if print_edges: 
    plt.subplot(121),plt.imshow(original_image,cmap = 'gray')
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122),plt.imshow(edges,cmap = 'gray')
    plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
    plt.show()

f2c.Visualizer.figure(200)
f2c.Visualizer.plot(cells)
f2c.Visualizer.plot(headlands)
f2c.Visualizer.plot(path)
f2c.Visualizer.show()
