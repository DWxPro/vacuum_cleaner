import os
import xacro

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription


from ament_index_python.packages import get_package_share_directory
from launch.actions import (EmitEvent, LogInfo,RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ComposableNode, ParameterFile

from launch.substitutions import TextSubstitution
from launch.actions import LogInfo

import yaml

def generate_launch_description():

    use_sim_time        = True
    package_name        = 'vacuum_cleaner'
    urdf_name           = 'robot.urdf.xacro'
    world_file_name     = 'my_world.world'
    slam_file_name      = 'slam.yaml'
    rviz_file_name      = 'presettings.rviz'
    joystick_file_name  = 'joystick.yaml'
    twist_mux_file_name = 'twist_mux.yaml'
    map_file_name       = "my_map.yaml"
    nav2_file_name      = 'nav2.yaml'

    path_package_share      = FindPackageShare(package=package_name).find(package_name)
    path_gazebo_ros_share   = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    use_sim_time_text   = "true" if use_sim_time else "false"

    robot_description   = xacro.process_file(os.path.join(path_package_share,'description',urdf_name), mappings = {'sim_mode': use_sim_time_text}).toxml()
    rviz_settings       = os.path.join(path_package_share,'config',rviz_file_name)
    slam_settings       = os.path.join(path_package_share,'config',slam_file_name)
    joystick_settings   = os.path.join(path_package_share ,'config',joystick_file_name)
    twist_mux_settings  = os.path.join(path_package_share,'config',twist_mux_file_name)
    world_settings      = os.path.join(path_package_share, 'worlds', world_file_name)
    map_file            = os.path.join(path_package_share,'maps',map_file_name)
    nav2_settings_raw   = os.path.join(path_package_share,'config',nav2_file_name)

    # controller_manager
    # running inside gazebo

    # diffbot_base_controller
    diffbot_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    # sweeper_controller
    sweeper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sweeper_controller", "-c", "/controller_manager"],
    )

    # gpio_simulation

    # robot_state_publisher (tf)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )
    
    # visualize robot
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_settings],
    )
    
    # slam_toolbox
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    declare_slam_parameters = DeclareLaunchArgument(
        'slam_params_file', default_value = slam_settings)

    slam_toolbox_node = LifecycleNode(
        parameters=[slam_params_file,{'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )
    
    # localication
    param_substitutions = {
        'use_sim_time': use_sim_time_text,
        'yaml_filename': map_file}

    nav2_settings = ParameterFile(
        RewrittenYaml(
            source_file=nav2_settings_raw,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    nav2_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}])

    nav2_map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn_delay=2.0,
        parameters=[nav2_settings],
        remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')])
    
    nav2_amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn_delay=2.0,
        parameters=[nav2_settings],
        remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')])

    # naviation


    # twist_mux
    twist_mux_node = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_settings, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
    )
    
    # joystick
    joystick_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joystick_settings, {'use_sim_time': use_sim_time}]
    )

    teleop_twist_joy_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joystick_settings, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
    )

    # gazebo
    gazebo_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    path_gazebo_ros_share, 'launch', 'gazebo.launch.py')])
    )

    # spawn entity
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description','-entity', 'robot_vacuum_cleaner'],
        output='screen'
    )

    # world
    declare_gazebo_world = DeclareLaunchArgument(
        name='world',
        default_value= world_settings,
        description='Full path to the world model file to load'
    )

    # create launch description
    ld = LaunchDescription()
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(declare_gazebo_world)
    ld.add_action(gazebo_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(diffbot_base_controller_spawner)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(sweeper_controller_spawner)
    #ld.add_action(declare_slam_parameters)
    #ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)
    ld.add_action(twist_mux_node)
    ld.add_action(joystick_node)
    ld.add_action(teleop_twist_joy_node) 
    ld.add_action(nav2_lifecycle_manager_node)
    ld.add_action(nav2_map_server_node)
    ld.add_action(nav2_amcl_node)

    return ld


