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



def generate_launch_description():

    package_name = 'vacuum_cleaner'
    urdf_name = 'robot.urdf.xacro'
    urdf_mappings = {'sim_mode': "true"}
    world_file_name = 'my_world.world'
    slam_mapping = False
    slam_mapping_file_name = 'slam_mapping.yaml'
    slam_localization_file_name = 'slam_localization.yaml'
    rviz_file_name = 'presettings.rviz'
    joystick_file_name = 'joystick.yaml'
    twist_mux_file_name = 'twist_mux.yaml'

    path_package_share = FindPackageShare(package=package_name).find(package_name)
    path_gazebo_ros_share = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    robot_description = xacro.process_file(os.path.join(path_package_share,'description',urdf_name), mappings = urdf_mappings).toxml()
    rviz_settings = os.path.join(path_package_share,'config',rviz_file_name)
    if slam_mapping:
        slam_settings = os.path.join(path_package_share,'config',slam_mapping_file_name)
    else:
        slam_settings = os.path.join(path_package_share,'config',slam_localization_file_name)
    joystick_settings = os.path.join(path_package_share ,'config',joystick_file_name)
    twist_mux_settings = os.path.join(path_package_share,'config',twist_mux_file_name)

    world_settings = os.path.join(path_package_share, 'worlds', world_file_name)

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
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
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
        'slam_params_file', default_value=slam_settings)

    slam_toolbox_node = LifecycleNode(
        parameters=[slam_params_file,{'use_sim_time': True}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )
       
    # twist_mux
    twist_mux_node = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_settings, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
    )
    
    # joystick
    joystick_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joystick_settings, {'use_sim_time': True}]
    )

    teleop_twist_joy_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joystick_settings, {'use_sim_time': True}],
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
    #ld.add_action(declare_gazebo_world)
    ld.add_action(gazebo_node)
    ld.add_action(spawn_entity_node)
    ld.add_action(diffbot_base_controller_spawner)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(sweeper_controller_spawner)
    #ld.add_action(declare_slam_parameters)
    #ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)
    #ld.add_action(twist_mux_node)
    #ld.add_action(joystick_node)
    #ld.add_action(teleop_twist_joy_node)

    return ld
