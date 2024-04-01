#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import numpy as np
import math

poses = [[-1.37814, 0.971746, 1.22173], [-1.5, 0.198374, 4.36332], [-1.12868, 0.78, 1.22173], [-1.7, -0.789692, 4.36332], [-1.05373, 0.547338, 1.22173], 
[-1.63776, -1.49584, 4.36332], [-0.849982, 0.23, 1.22173], [-1.36367, -1.61993, 4.36332], [-0.558926, 0.15253, 1.22173], [-1.07624, -1.70736, 4.36332], 
[-0.271495, 0.0650988, 1.22173], [-0.670598, -1.47, 4.36332], [0.016785, -0.02, 1.22173], [-0.351345, -1.47, 4.36332], [0.236656, -0.293052, 1.22173], 
[-0.0502903, -1.52, 4.36332], [0.470717, -0.527114, 1.22173], [0.250764, -1.57, 4.36332], [0.608841, -1.02476, 4.36332], [0.6, -1.04905, 1.22173], ]


def set_initial_pose(navigator):
    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -0.783
    initial_pose.pose.position.y = -0.617
    initial_pose.pose.orientation.z = -0.149
    initial_pose.pose.orientation.w =  0.99
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

def handle_result(navigator):
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

def radians_to_quaternion(rad):
    w = np.cos(rad * 0.5)
    z = np.sin(rad * 0.5)

    return w, z

def degrees_to_quaternion(degrees):
    rad = degrees * np.pi / 180.0

    w = np.cos(rad * 0.5)
    z = np.sin(rad * 0.5)

    return w, z

def nav_to_pose():
    rclpy.init()
    navigator = BasicNavigator()

    #set_initial_pose(navigator)

    w, z = degrees_to_quaternion(90)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = -0.80
    goal_pose.pose.position.y = -0.80
    goal_pose.pose.orientation.w = w
    goal_pose.pose.orientation.z = z

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

    #handle_result(navigator)
    #navigator.lifecycleShutdown()
    #exit(0)

def waypoint_follower(poses):
 
    rclpy.init()
    navigator = BasicNavigator()
    #set_initial_pose(navigator)

    goal_poses = []
    for i in range(0,len(poses)-1):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = poses[i][0]
        goal_pose.pose.position.y = poses[i][1]

        if i < len(poses)-1:
            dx = poses[i+1][0] - poses[i][0]
            dy = poses[i+1][1] - poses[i][1]
            w,z = radians_to_quaternion(math.atan(dy / dx))

        goal_pose.pose.orientation.z = z
        goal_pose.pose.orientation.w = w
        goal_poses.append(goal_pose)


    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():

        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Executing current waypoint: '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(goal_poses))
            )

    #handle_result(navigator)
    #navigator.lifecycleShutdown()
    #exit(0)

def nav_through_poses(poses):
    rclpy.init()
    navigator = BasicNavigator()
    #set_initial_pose(navigator)
    
    goal_poses = []
    for i in range(0,len(poses)-1):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = poses[i][0]
        goal_pose.pose.position.y = poses[i][1]

        if i < len(poses)-1:
            dx = poses[i+1][0] - poses[i][0]
            dy = poses[i+1][1] - poses[i][1]
            w,z = radians_to_quaternion(math.atan(dy / dx))

        goal_pose.pose.orientation.z = z
        goal_pose.pose.orientation.w = w
        goal_poses.append(goal_pose)


    navigator.goThroughPoses(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

    #handle_result(navigator)
    #navigator.lifecycleShutdown()
    #exit(0)

def follow_path(poses):
    rclpy.init()
    navigator = BasicNavigator()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -0.783
    initial_pose.pose.position.y = -0.617
    initial_pose.pose.orientation.z = -0.149
    initial_pose.pose.orientation.w =  0.99
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()
    
    goal_poses = []
    for i in range(0,len(poses)-1):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = poses[i][0]
        goal_pose.pose.position.y = poses[i][1]

        if i < len(poses)-1:
            dx = poses[i+1][0] - poses[i][0]
            dy = poses[i+1][1] - poses[i][1]
            w,z = radians_to_quaternion(math.atan(dy / dx))

        goal_pose.pose.orientation.z = z
        goal_pose.pose.orientation.w = w
        goal_poses.append(goal_pose)

    # Get the path, smooth it
    path = navigator.getPathThroughPoses(initial_pose, goal_poses)
    #smoothed_path = navigator.smoothPath(path)

    # Follow path
    navigator.followPath(path)

    # Do something with the feedback
    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated distance remaining to goal position: '
                + '{0:.3f}'.format(feedback.distance_to_goal)
                + '\nCurrent speed of the robot: '
                + '{0:.3f}'.format(feedback.speed)
            )

    #handle_result(navigator)
    #navigator.lifecycleShutdown()
    #exit(0)

if __name__ == '__main__':
    #nav_to_pose()
    waypoint_follower(poses)
    #nav_through_poses(poses)
    #follow_path(poses)