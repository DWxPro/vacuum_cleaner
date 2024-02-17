import os
import xacro

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription

def generate_launch_description():

    package_name = 'vacuum_cleaner'
    urdf_name = 'robot.urdf.xacro'
    urdf_mappings = {'sim_mode': "true"}
    world_file_name = 'my_world.world'
    slam_file_name = 'slam.yaml'
    rviz_file_name = 'presettings.rviz'

    path_package_share = FindPackageShare(package=package_name).find(package_name)
    path_gazebo_ros_share = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    robot_description = xacro.process_file(os.path.join(path_package_share,'description',urdf_name), mappings = urdf_mappings).toxml()
    world_settings = os.path.join(path_package_share, 'worlds', world_file_name)
    rviz_settings = os.path.join(path_package_share,'config',rviz_file_name)
    slam_settings = os.path.join(path_package_share,'config',slam_file_name)

    # controller_manager
    # running inside gazebo

    # diffbot_base_controller
    diffbot_base_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # joint_state_broadcaster
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )
    
    # robot_state_publisher (tf)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        remappings=[("/vacuum_cleaner_controller/cmd_vel_unstamped", "/cmd_vel")]
    )

    # visualize robot
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_settings],
    )

    # gazebo
    gazebo_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    path_gazebo_ros_share, 'launch', 'gazebo.launch.py')]),
             )

    # slam_toolbox
    slam_toolbox_node = Node(
        parameters=[{'params_file': slam_settings,'use_sim_time': True}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    # spawn entity
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description','-entity', 'robot_vacuum_cleaner'],
        output='screen'
    )

    # world
    declare_world_argument = DeclareLaunchArgument(
        name='world',
        default_value= world_settings,
        description='Full path to the world model file to load')
    

    # create launch description
    ld = LaunchDescription()
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(declare_world_argument)
    ld.add_action(gazebo_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(diffbot_base_controller_node)
    ld.add_action(joint_state_broadcaster_node)
    ld.add_action(slam_toolbox_node)    
    ld.add_action(rviz_node)

    return ld
