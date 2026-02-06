from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch file for RRT path planning only"""
    
    # Packages
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # World file with EMBEDDED robot
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('rl_maze_nav'), 'worlds', 'labirent.world'),
        description='Full path to world file'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'verbose': 'true',
            'pause': 'false'
        }.items()
    )
    
    # RRT Planner Node
    rrt_node = Node(
        package='rl_maze_nav',
        executable='rrt_planner',
        name='rrt_maze_nav',
        output='screen',
        parameters=[{'use_sim_time': True}],
        prefix='bash -c "sleep 7 && $0 $@"'
    )
    
    return LaunchDescription([
        world_arg,
        gui_arg,
        gazebo,
        rrt_node
    ])

