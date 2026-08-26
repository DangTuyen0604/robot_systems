from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    gui = LaunchConfiguration('gui')
    show_debug = LaunchConfiguration('show_debug')
    initial_mode = LaunchConfiguration('initial_mode')
    simulation_config = os.path.join(
        get_package_share_directory('DATN'), 'config', 'simulation.yaml')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('DATN'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={'gui': gui}.items(),
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo graphical client'),
        DeclareLaunchArgument(
            'show_debug', default_value='false',
            description='Open rqt_image_view'),
        DeclareLaunchArgument(
            'initial_mode', default_value='AUTO',
            description='Initial command mux mode: AUTO or ESTOP'),

        LogInfo(msg='=== Khoi dong he thong MO PHONG (Gazebo) ==='),

        gazebo_launch,

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'image_transport', 'republish',
                'raw', 'compressed',
                '--ros-args',
                '-r', 'in:=/camera/camera/image_raw',
                '-r', 'out/compressed:=/raw_image/compressed',
            ],
            output='screen'
        ),

        Node(
            package='decision_pkg',
            executable='command_mux_node',
            name='command_mux',
            output='screen',
            parameters=[simulation_config, {'initial_mode': initial_mode}],
        ),

        Node(
            package='decision_pkg',
            executable='sim_motor_bridge_node',
            name='sim_motor_bridge_node',
            output='screen',
            parameters=[simulation_config],
        ),

        Node(
            package='perception_pkg',
            executable='lane_detection_node',
            name='lane_node_instance',
            output='screen',
            parameters=[simulation_config]
        ),

        Node(
            package='perception_pkg',
            executable='traffic_sign_node',
            name='sign_node_instance',
            output='screen',
            parameters=[simulation_config]
        ),

        Node(
            package='decision_pkg',
            executable='decision_node',
            name='decision_node_instance',
            output='screen',
            parameters=[simulation_config],
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
            output='screen',
            condition=IfCondition(show_debug),
        ),
    ])
