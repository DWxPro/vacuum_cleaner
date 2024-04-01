import path_planing as pp
import drive_robot as dr

from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.duration import Duration


# https://github.com/ros-planning/navigation2/blob/main/nav2_simple_commander/nav2_simple_commander
# https://navigation.ros.org/commander_api/index.html
# https://navigation.ros.org/commander_api/index.html#examples-and-demos

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
home_map = pp.Map(map_image_path, map_yaml_path)

# init rooms
headlands_offset = 0.3      #[m]

living_room = pp.Room("living_room", home_map, colors_living_room, headlands_offset)
kitchen     = pp.Room("kitchen", home_map, colors_kitchen, headlands_offset)
badroom     = pp.Room("badroom", home_map, colors_badroom, headlands_offset)
corridor    = pp.Room("corridor", home_map, colors_corridor, headlands_offset)
toilet      = pp.Room("toilet", home_map, colors_toilet, headlands_offset)
bathroom    = pp.Room("bathroom", home_map, colors_bathroom, headlands_offset)
hallway     = pp.Room("hallway", home_map, colors_hallway, headlands_offset)
workspace   = pp.Room("workspace", home_map, colors_workspace, headlands_offset)

# init robot
vacuum_cleaner = pp.Robot(robot_width, operation_width)

def main():
    living_room.calculate_path(vacuum_cleaner, 110, 3)
    poses = pp.get_poses_from_path(living_room.path)
    pp.print_room_path(living_room)

    rclpy.init()
    navigator = BasicNavigator()

    dr.set_initial_pose(navigator)
    dr.waypoint_follower(poses)


if __name__ == '__main__':
    main()


