import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Set up the path to the URDF file
    urdf_file = 'simple_robot_drive.urdf'
    pkg_share_path = get_package_share_directory('simple_robot_description')
    urdf_path = os.path.join(pkg_share_path, 'urdf', urdf_file)
    world_file = os.path.join(pkg_share_path, 'worlds', 'obstacle_course_world.world')

    # Load the URDF into a parameter using xacro
    #robot_description = Command(['xacro', urdf_path])

    # Load the URDF file content directly
    try:
        with open(urdf_path, 'r') as urdf_file_handle:
            robot_description = urdf_file_handle.read()
    except FileNotFoundError:
        print(f"ERROR: URDF file not found at {urdf_path}")
        raise
    except PermissionError:
        print(f"ERROR: Permission denied for URDF file at {urdf_path}")
        raise
    
    # Declare a launch argument for use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Set GAZEBO_MODEL_PATH and GAZEBO_PLUGIN_PATH to include our workspace
    # This is critical for Gazebo to find the custom models and plugins.
    gazebo_models_path = os.path.join(pkg_share_path, 'models')
    gazebo_plugins_path = os.path.join(pkg_share_path, 'lib')
    
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':' + gazebo_models_path]
    )

    set_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=[os.environ.get('GAZEBO_PLUGIN_PATH', ''), ':' + gazebo_plugins_path]
    )
    
    # Start the Gazebo server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file,
                          'verbose': 'true',
                          'use_sim_time': use_sim_time}.items()
    )

    # Spawn the robot into Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'simple_robot_drive'],
        output='screen'
    )
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
    )

    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        remappings=[('/diff_drive_controller/cmd_vel_unstamped', '/cmd_vel')]
    )

    return LaunchDescription([
        # Add the environment variables
        set_model_path,
        set_plugin_path,
        
        # Add the launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time.'),
        
        # Gazebo Launch
        gzserver_cmd,
        
        # Robot State Publisher
        robot_state_publisher_node,
        
        # Spawn the robot
        spawn_entity_node,
        
        # Spawners to load controllers
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])
