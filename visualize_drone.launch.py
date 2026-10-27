import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('airmouse_autonomy')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'airmouse.urdf')
    rviz_path = os.path.join(pkg_dir, 'rviz', 'airmouse_gcs.rviz')

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='airmouse_autonomy',
            executable='arena_simulator',
            output='screen'
        ),
        Node(
            package='airmouse_autonomy',
            executable='mission_manager',
            output='screen'
        ),
        Node(
            package='airmouse_autonomy',
            executable='survivor_detector',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_path],
            output='screen'
        )
    ])