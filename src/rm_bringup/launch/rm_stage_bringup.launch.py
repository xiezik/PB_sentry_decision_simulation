from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument,OpaqueFunction,SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    this_directory = get_package_share_directory('rm_bringup')
    map_arg = DeclareLaunchArgument(
        'map',
        default_value='icra2020',
        description='Map argument')

    use_sim_time_param = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')

    def stage_world_configuration(context):
        file = os.path.join(
            this_directory,
            'worlds',  # Convert LaunchConfiguration to string
            'icra2020.world')
        return [SetLaunchConfiguration('world_file', file)]

    stage_world_configuration_arg = OpaqueFunction(function=stage_world_configuration)

    rviz_config_dir = os.path.join(
        get_package_share_directory('rm_bringup'),
        'config',
        'rviz',
        'stage_ros.rviz')

    return LaunchDescription([
        map_arg,
        use_sim_time_param,
        stage_world_configuration_arg,
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            parameters=[{
                "world_file": [LaunchConfiguration('world_file')]}],
            remappings=[("/base_scan","/scan")],
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            # arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': [LaunchConfiguration('use_sim_time')]}],
            output='screen')
    ])
