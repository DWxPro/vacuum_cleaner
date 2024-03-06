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
    slam_mapping = True
    slam_mapping_file_name = 'slam_mapping.yaml'
    slam_localization_file_name = 'slam_localization.yaml'
    rviz_file_name = 'presettings.rviz'
    joystick_file_name = 'joystick.yaml'
    twist_mux_file_name = 'twist_mux.yaml'
    controller_settings_file_name = 'controllers.yaml'
    rplidar_port = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'

    path_package_share = FindPackageShare(package=package_name).find(package_name)

    robot_description = xacro.process_file(os.path.join(path_package_share,'description',urdf_name), mappings = urdf_mappings).toxml()
    rviz_settings = os.path.join(path_package_share,'config',rviz_file_name)
    if slam_mapping:
        slam_settings = os.path.join(path_package_share,'config',slam_mapping_file_name)
    else:
        slam_settings = os.path.join(path_package_share,'config',slam_localization_file_name)
    joystick_settings = os.path.join(path_package_share ,'config',joystick_file_name)
    twist_mux_settings = os.path.join(path_package_share,'config',twist_mux_file_name)
    
    controller_settings = os.path.join(path_package_share,'config',controller_settings_file_name)

    # controller_manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_settings],
        output="both",
    )

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
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # range_sensor_broadcaster_node
    range_sensor_broadcaster_left_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["range_sensor_broadcaster_left", "--controller-manager", "/controller_manager"],
    )
    range_sensor_broadcaster_front_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["range_sensor_broadcaster_front", "--controller-manager", "/controller_manager"]
    )
    range_sensor_broadcaster_right_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["range_sensor_broadcaster_right", "--controller-manager", "/controller_manager"]
    )

    # robot_state_publisher (tf)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
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
    slam_toolbox_node = Node(
        parameters=[{'params_file': slam_settings,'use_sim_time': False}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )
    
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
    
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        parameters=[{'serial_port': rplidar_port, 'frame_id': 'lidar_frame', 'angle_compensate': True, 'scanmode': 'Standard'}]
    )

    # create launch description
    ld = LaunchDescription()

    ld.add_action(control_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(diffbot_base_controller_node)
    ld.add_action(joint_state_broadcaster_node)
    ld.add_action(range_sensor_broadcaster_left_node)
    ld.add_action(range_sensor_broadcaster_front_node)
    ld.add_action(range_sensor_broadcaster_right_node)
    #ld.add_action(slam_toolbox_node)
    #ld.add_action(rviz_node)
    ld.add_action(twist_mux_node)
    ld.add_action(joystick_node)
    ld.add_action(teleop_twist_joy_node)
    #ld.add_action(rplidar_node)

    return ld
