import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription,LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition
from nav2_common.launch import ReplaceString, RewrittenYaml

def generate_launch_description():
    # Get the launch directory
    rm_bringup_dir = get_package_share_directory('rm_bringup')
    rm_multi_dir = get_package_share_directory('rm_multiage')
    navigation2_launch_dir = os.path.join(get_package_share_directory('rm_navigation'), 'launch')

    # Create the launch configuration variables
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    nav2_multirobot_params_file = LaunchConfiguration('params_file')
    robot_num = LaunchConfiguration('robot_num')
    use_nav_rviz = LaunchConfiguration('nav_rviz')

    params_file = ReplaceString(
        source_file=nav2_multirobot_params_file,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace),
    )

    ################################## slam_toolbox parameters start ##################################
    slam_toolbox_map_dir = PathJoinSubstitution([rm_bringup_dir, 'maps', world])
    slam_toolbox_localization_file_dir = os.path.join(rm_bringup_dir, 'config', 'simulation', 'mapper_params_localization_sim.yaml')
    slam_toolbox_mapping_file_dir = os.path.join(rm_bringup_dir, 'config', 'simulation', 'mapper_params_online_async_sim.yaml')
    ################################### slam_toolbox parameters end ###################################

    ################################### navigation2 parameters start ##################################
    nav2_map_dir = PathJoinSubstitution([rm_multi_dir, 'maps', world]), ".yaml"
    multi_loaclization_dir = os.path.join(rm_bringup_dir, 'config', 'simulation', 'multiage_localization.yaml')
    ################################### navigation2 parameters end ####################################

    
    # Declare launch options
    declare_robot_num_cmd = DeclareLaunchArgument(
        'robot_num',
        default_value='robot_0',  # 假设默认值为0，可以根据实际情况调整
        description='Number of the robot for initialization')
    
    declare_param_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(rm_bringup_dir, 'config', 'simulation', 'nav2_multirobot_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='rmua',
        description='World file name')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', 
        default_value='robbot_0', 
        description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='True',
        description='Whether to apply a namespace to the navigation stack'
    )

    declare_nav_rviz_cmd = DeclareLaunchArgument(
        'nav_rviz',
        default_value='True',
        description='Visualize navigation2 if true')

    declare_mode_cmd = DeclareLaunchArgument(
        'mode',
        default_value='',
        description='Choose mode: nav, mapping')

    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='',
        description='Choose localization method: slam_toolbox, amcl')
    
    #load inital pose from multiage_localization.yaml
    multi_params = yaml.safe_load(open(multi_loaclization_dir))

    start_localization_group = GroupAction(
        condition = LaunchConfigurationEquals('mode', 'nav'),
        actions=[
            Node(
                condition = LaunchConfigurationEquals('robot_num', 'robot_0'),
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_toolbox_localization_file_dir,
                    {'use_sim_time': use_sim_time,
                    'map_file_name': slam_toolbox_map_dir,
                    'map_start_pose': [
                        float(multi_params['initial_pose_x_robot_0']),
                        float(multi_params['initial_pose_y_robot_0']),
                        float(multi_params['initial_pose_a_robot_0'])
                    ]}
                ],
            ),

            Node(
                condition = LaunchConfigurationEquals('robot_num', 'robot_1'),
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_toolbox_localization_file_dir,
                    {'use_sim_time': use_sim_time,
                    'map_file_name': slam_toolbox_map_dir,
                    'map_start_pose': [
                        float(multi_params['initial_pose_x_robot_1']),
                        float(multi_params['initial_pose_y_robot_1']),
                        float(multi_params['initial_pose_a_robot_1'])
                    ]}
                ],
            ),

            Node(
                condition = LaunchConfigurationEquals('robot_num', 'robot_2'),
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_toolbox_localization_file_dir,
                    {'use_sim_time': use_sim_time,
                    'map_file_name': slam_toolbox_map_dir,
                    'map_start_pose': [
                        float(multi_params['initial_pose_x_robot_2']),
                        float(multi_params['initial_pose_y_robot_2']),
                        float(multi_params['initial_pose_a_robot_2'])
                    ]}
                ],
            ),

            Node(
                condition = LaunchConfigurationEquals('robot_num', 'robot_3'),
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[
                    slam_toolbox_localization_file_dir,
                    {'use_sim_time': use_sim_time,
                    'map_file_name': slam_toolbox_map_dir,
                    'map_start_pose': [
                        float(multi_params['initial_pose_x_robot_3']),
                        float(multi_params['initial_pose_y_robot_3']),
                        float(multi_params['initial_pose_a_robot_3'])
                    ]}
                ],
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'map_server_launch.py')),
                condition = LaunchConfigurationNotEquals('localization', 'slam_toolbox'),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': nav2_map_dir,
                    'params_file': params_file,
                    'container_name': 'nav2_container'}.items())
        ]
    )

    start_mapping = Node(
        condition = LaunchConfigurationEquals('mode', 'mapping'),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_toolbox_mapping_file_dir,
            {'use_sim_time': use_sim_time,}
        ],
    )

    start_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation2_launch_dir, 'bringup_rm_navigation.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': nav2_map_dir,
            'params_file': params_file,
            'nav_rviz': use_nav_rviz}.items()
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_robot_num_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_param_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_nav_rviz_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_localization_cmd)

    ld.add_action(start_localization_group)
    ld.add_action(start_mapping)
    ld.add_action(start_navigation2)

    return ld
