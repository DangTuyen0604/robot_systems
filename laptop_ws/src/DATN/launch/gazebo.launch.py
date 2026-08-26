import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    gui = LaunchConfiguration('gui')
    
    package_name = 'DATN'
    urdf_file_name = 'robot_des.urdf'

    
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', urdf_file_name)
    package_share = get_package_share_directory(package_name)

    # Allow Gazebo Classic to resolve file://materials/... used by the
    # high-resolution traffic-sign textures in the world file.
    gazebo_resource_path = os.pathsep.join(filter(None, [
        package_share,
        os.environ.get('GAZEBO_RESOURCE_PATH', ''),
        '/usr/share/gazebo-11',
    ]))
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=gazebo_resource_path,
    )

    
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
        launch_arguments={'world': world_path, 'gui': gui}.items()
    )

    
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot', '-z', '0.05'],
        output='screen'
    )

    
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo graphical client'),
        set_gazebo_resource_path,
        robot_state_publisher_node,
        gazebo_launch,
        spawn_robot_node
    ])
