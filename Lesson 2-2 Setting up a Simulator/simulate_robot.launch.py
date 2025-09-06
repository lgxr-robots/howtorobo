from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_robot_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'simple_robot.urdf')
    world_file = os.path.join(pkg_dir, 'worlds', 'simple_world.world')

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Spawn the robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_robot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        ),
    ])
