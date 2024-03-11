import os
import xacro

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LifecycleNode
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile

def generate_launch_description():

    use_sim_time            = False
    package_name            = 'vacuum_cleaner'
    urdf_name               = 'robot.urdf.xacro'
    joystick_file_name      = 'joystick.yaml'
    twist_mux_file_name     = 'twist_mux.yaml'
    slam_file_name          = 'slam.yaml'
    map_file_name           = "my_map.yaml"
    nav2_file_name          = 'nav2.yaml'
    rplidar_port            = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
    controller_file_name   = 'controllers.yaml'

    path_package_share  = FindPackageShare(package=package_name).find(package_name)

    use_sim_time_text   = "true" if use_sim_time else "false"

    robot_description   = xacro.process_file(os.path.join(path_package_share,'description',urdf_name), mappings = {'sim_mode': use_sim_time_text}).toxml()
    slam_settings       = os.path.join(path_package_share,'config',slam_file_name)
    joystick_settings   = os.path.join(path_package_share ,'config',joystick_file_name)
    twist_mux_settings  = os.path.join(path_package_share,'config',twist_mux_file_name)
    map_file            = os.path.join(path_package_share,'maps',map_file_name)
    nav2_settings_raw   = os.path.join(path_package_share,'config',nav2_file_name)
    controller_settings = os.path.join(path_package_share,'config',controller_file_name)

    # controller_manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controller_settings],
        output="both",
    )

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
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # range_sensor_broadcaster
    range_sensor_broadcaster_left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["range_sensor_broadcaster_left", "--controller-manager", "/controller_manager"],
    )
    range_sensor_broadcaster_front_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["range_sensor_broadcaster_front", "--controller-manager", "/controller_manager"]
    )
    range_sensor_broadcaster_right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["range_sensor_broadcaster_right", "--controller-manager", "/controller_manager"]
    )

    # sweeper_controller
    sweeper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sweeper_controller", "-c", "/controller_manager"],
    )

    # gpio_controller
    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gpio_controller", "-c", "/controller_manager"],
    )

    # robot_state_publisher (tf)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    # slam_toolbox
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    declare_slam_parameters = DeclareLaunchArgument(
        'slam_params_file', default_value=slam_settings)

    slam_toolbox_node = LifecycleNode(
        parameters=[slam_params_file,{'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )
    

    # nav2_lifecycle_manager
    nav2_lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server',
                                    'amcl',
                                    'controller_server',
                                    'smoother_server',
                                    'planner_server',
                                    'behavior_server',
                                    'bt_navigator',
                                    'waypoint_follower',
                                    'velocity_smoother']}])
    
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
    nav2_controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        respawn_delay=2.0,
        parameters=[nav2_settings],
        remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')])
    
    nav2_smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        respawn_delay=2.0,
        parameters=[nav2_settings],
        remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')])
    
    nav2_planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        respawn_delay=2.0,
        parameters=[nav2_settings],
        remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')])
    
    nav2_behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        respawn_delay=2.0,
        parameters=[nav2_settings],
        remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')])
    
    nav2_bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        respawn_delay=2.0,
        parameters=[nav2_settings],
        remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')])
    
    nav2_waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        respawn_delay=2.0,
        parameters=[nav2_settings],
        remappings=[('/tf', 'tf'),('/tf_static', 'tf_static')])
    
    nav2_velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn_delay=2.0,
        parameters=[nav2_settings],
        remappings=[('/tf', 'tf'),('/tf_static', 'tf_static'),('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')])

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
        parameters=[joystick_settings, {'use_sim_time': use_sim_time}],
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joystick_settings, {'use_sim_time': use_sim_time}],
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
    ld.add_action(twist_mux_node)
    ld.add_action(joystick_node)
    ld.add_action(teleop_twist_joy_node)
    ld.add_action(rplidar_node)

    ld.add_action(robot_state_publisher_node)
    ld.add_action(diffbot_base_controller_spawner)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(range_sensor_broadcaster_left_spawner)
    ld.add_action(range_sensor_broadcaster_front_spawner)
    ld.add_action(range_sensor_broadcaster_right_spawner)
    ld.add_action(sweeper_controller_spawner)
    ld.add_action(gpio_controller_spawner)

    #ld.add_action(declare_slam_parameters)
    #ld.add_action(slam_toolbox_node)

    ld.add_action(nav2_lifecycle_manager_node)
    ld.add_action(nav2_map_server_node)
    ld.add_action(nav2_amcl_node)
    ld.add_action(nav2_controller_server_node)
    ld.add_action(nav2_smoother_server_node)
    ld.add_action(nav2_planner_server_node)
    ld.add_action(nav2_behavior_server_node)
    ld.add_action(nav2_bt_navigator_node)
    ld.add_action(nav2_waypoint_follower_node)
    ld.add_action(nav2_velocity_smoother_node)

    return ld
