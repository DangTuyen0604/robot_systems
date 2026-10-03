from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    gui = LaunchConfiguration('gui')
    show_debug = LaunchConfiguration('show_debug')
    initial_mode = LaunchConfiguration('initial_mode')
    signs = LaunchConfiguration('signs')
    # Perception, decision and safety nodes use exactly the same parameters
    # as the Gazebo simulation; only the simulator is replaced.
    simulation_config = os.path.join(
        get_package_share_directory('DATN'), 'config', 'simulation.yaml')
    mujoco_config = os.path.join(
        get_package_share_directory('mujoco_sim_pkg'), 'config', 'mujoco_sim.yaml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Open the MuJoCo viewer window'),
        DeclareLaunchArgument(
            'show_debug', default_value='false',
            description='Open rqt_image_view'),
        DeclareLaunchArgument(
            'initial_mode', default_value='AUTO',
            description='Initial command mux mode: AUTO or ESTOP'),
        DeclareLaunchArgument(
            'signs', default_value='true',
            description='Run the YOLO traffic-sign node (needs the model file)'),

        LogInfo(msg='=== Khoi dong he thong MO PHONG (MuJoCo) ==='),

        Node(
            package='mujoco_sim_pkg',
            executable='mujoco_sim_node',
            name='mujoco_sim_node',
            output='screen',
            parameters=[mujoco_config, {'gui': gui}],
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
            parameters=[simulation_config],
            condition=IfCondition(signs),
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
