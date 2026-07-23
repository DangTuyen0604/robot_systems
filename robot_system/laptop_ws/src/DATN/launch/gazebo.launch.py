import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = 'DATN'
    urdf_file_name = 'robot_des.urdf'

    
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', urdf_file_name)

    
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

   
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'datn_track_world.world')

    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot', '-z', '0.05'],
        output='screen'
    )

    
    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot_node
    ])