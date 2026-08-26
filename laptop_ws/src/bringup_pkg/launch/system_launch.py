from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ── Launch Arguments ──────────────────────────────────────────────────────
    publish_image_arg = DeclareLaunchArgument(
        'publish_image',
        default_value='true',
        description='Publish debug image topics. Đặt false khi deploy để tiết kiệm CPU'
    )

    return LaunchDescription([
        publish_image_arg,
        LogInfo(msg='=== Khởi động hệ thống robot ==='),

        # ── Node 1: Lane Detection (perception_pkg) ───────────────────────────
        Node(
            package='perception_pkg',
            executable='lane_detection_node',
            name='lane_node_instance',
            output='screen',
            parameters=[{
                'lost_lane_recovery_enabled': False,
                'lost_lane_recovery_frames': 10,
            }]
        ),

        # ── Node 2: Traffic Sign + Light Detection (perception_pkg) ───────────
        Node(
            package='perception_pkg',
            executable='traffic_sign_node',
            name='sign_node_instance',
            output='screen',
            parameters=[{
                'sign_conf':     0.50,
                'light_conf':    0.50,
                'imgsz':         640,
                'process_every_n_frames': 2,
                'publish_image': LaunchConfiguration('publish_image'),
            }]
        ),

        # ── Node 3: Decision (decision_pkg) ───────────────────────────────────
        Node(
            package='decision_pkg',
            executable='decision_node',
            name='decision_node_instance',
            output='screen',
            parameters=[{
                'stop_duration': 2.0,
                'sign_timeout':  1.5,
                'lane_timeout':  0.3,
            }]
        ),

        # ── rqt_image_view: cửa sổ xem ảnh debug ─────────────────────────────
        Node(
            package='decision_pkg',
            executable='command_mux_node',
            name='command_mux',
            output='screen',
            parameters=[{'initial_mode': 'ESTOP', 'command_timeout': 0.3}]
        ),
    ])
