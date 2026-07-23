from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('DATN'),
                'launch', 'gazebo.launch.py'
            )
        )
    )

    return LaunchDescription([

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
            executable='sim_motor_bridge_node',
            name='sim_motor_bridge_node',
            output='screen',
        ),

        Node(
            package='perception_pkg',
            executable='lane_detection_node',
            name='lane_node_instance',
            output='screen',
            parameters=[{
                'seg_conf': 0.25,
                'seg_iou': 0.45,
                'seg_imgsz': 640,
                'lane_class_id': 0,
            }]
        ),

        Node(
            package='perception_pkg',
            executable='traffic_sign_node',
            name='sign_node_instance',
            output='screen',
            parameters=[{
                'imgsz': 320,
                'process_every_n_frames': 2,
                'camera_f': 530.0,
                'camera_dy': 1.0,
                'camera_h': 0.26,
                'camera_alpha': 35.0,
                'camera_v0': 240.0,
            }]
        ),

        Node(
            package='decision_pkg',
            executable='decision_node',
            name='decision_node_instance',
            output='screen',
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
            output='screen'
        ),
    ])
