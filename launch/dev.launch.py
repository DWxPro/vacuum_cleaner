import os
import xacro

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    package_name = 'vacuum_cleaner'
    slam_mapping = True
    slam_mapping_file_name = 'slam_mapping.yaml'
    slam_localization_file_name = 'slam_localization.yaml'
    rviz_file_name = 'presettings.rviz'
    joystick_file_name = 'joystick.yaml'
    twist_mux_file_name = 'twist_mux.yaml'

    path_package_share = FindPackageShare(package=package_name).find(package_name)

    rviz_settings = os.path.join(path_package_share,'config',rviz_file_name)
    if slam_mapping:
        slam_settings = os.path.join(path_package_share,'config',slam_mapping_file_name)
    else:
        slam_settings = os.path.join(path_package_share,'config',slam_localization_file_name)
    joystick_settings = os.path.join(path_package_share ,'config',joystick_file_name)
    twist_mux_settings = os.path.join(path_package_share,'config',twist_mux_file_name)

    # visualize robot
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_settings],
    )

    # slam_toolbox
    slam_toolbox_node = Node(
        parameters=[{'params_file': slam_settings,'use_sim_time': False}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    # twist_mux
    twist_mux_node = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_settings, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
        )
    
    # joystick
    joystick_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joystick_settings, {'use_sim_time': False}],
         )

    teleop_twist_joy_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joystick_settings, {'use_sim_time': False}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )
    
    # create launch description
    ld = LaunchDescription()

    #ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)
    #ld.add_action(twist_mux_node)
    ld.add_action(joystick_node)
    ld.add_action(teleop_twist_joy_node)

    return ld
