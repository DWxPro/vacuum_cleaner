import os
import xacro

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    package_name = 'vacuum_cleaner'
    urdf_name = 'robot.urdf.xacro'
    urdf_mappings = {'sim_mode': "false"}
    rviz_file_name = 'presettings.rviz'
    controller_settings_file_name = 'diffbot_controllers.yaml'

    path_package_share = FindPackageShare(package=package_name).find(package_name)

    robot_description = xacro.process_file(os.path.join(path_package_share,'description',urdf_name), mappings = urdf_mappings).toxml()
    controller_settings = os.path.join(path_package_share,'config',controller_settings_file_name)
    rviz_settings = os.path.join(path_package_share,'config',rviz_file_name)

    # controller_manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_settings],
        output="both",
    )

    # diffbot_base_controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # robot_state_publisher (tf)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
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

    # delays
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )


    # create launch description
    ld = LaunchDescription()

    ld.add_action(control_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(delay_rviz)
    ld.add_action(delay_robot_controller_spawner)

    return ld
