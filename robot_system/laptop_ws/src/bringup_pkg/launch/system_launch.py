from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
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
                # Model & inference
                'input_size':          256,
                'lane_class_id':       0,
                'seg_threshold':       0.5,
                # ROI & steering
                'roi_start_ratio':     0.55,
                'steering_smooth':     0.65,
                'steering_scale':      1.0,
                'steer_medium_thresh': 0.25,
                'steer_hard_thresh':   0.50,
                # Speed
                'speed_default':       0.25,
                'speed_turn_medium':   0.18,
                'speed_turn_hard':     0.10,
                # Debug
                'process_every_n_frames': 1,
                'publish_image':       LaunchConfiguration('publish_image'),
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
                'imgsz':         320,
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
                'base_speed':    0.25,
                'stop_duration': 2.0,
                'sign_timeout':  1.5,
            }]
        ),

        # ── rqt_image_view: cửa sổ xem ảnh debug ─────────────────────────────
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
            output='screen'
        ),
    ])
