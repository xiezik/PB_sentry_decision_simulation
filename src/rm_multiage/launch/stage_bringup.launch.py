import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition

def generate_launch_description():
        # Get the launch directory
        rm_multiage_launch_dir = os.path.join(get_package_share_directory('rm_multiage'), 'launch')
        battlefile_params_file = os.path.join(get_package_share_directory('rm_multiage'), 'worlds', 'battlefield.yaml')

        battlefile_params = yaml.safe_load(open(battlefile_params_file))
        # Create the launch configuration variables
        world = LaunchConfiguration('world')
        use_sim_time = LaunchConfiguration('use_sim_time')
        use_nav_rviz = LaunchConfiguration('nav_rviz')

        # Declare launch options
        declare_use_sim_time_cmd = DeclareLaunchArgument(
                'use_sim_time',
                default_value='True',
                description='Use simulation (Gazebo) clock if true'
        )

        declare_nav_rviz_cmd = DeclareLaunchArgument(
                'nav_rviz',
                default_value='True',
                description='Visualize navigation2 if true'
        )




        IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(rm_multiage_launch_dir,'multi_stageros.launch.py')),
                )


        robot_0_bringup = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(rm_multiage_launch_dir,'launch_robot.launch.py')),
                        launch_arguments = {
                        'namespace': "robot_0",
                        'use_namespace': 'True',
                        'use_sim_time': use_sim_time,
                        'robot_num': "robot_0"}.items()
                )
        robot_1_bringup = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(rm_multiage_launch_dir,'launch_robot.launch.py')),
                        launch_arguments = {
                        'namespace': "robot_1",
                        'use_namespace': 'True',
                        'use_sim_time': use_sim_time,
                        'robot_num': "robot_1"}.items()
                )
        # robot_2_bringup = IncludeLaunchDescription(
        #                 PythonLaunchDescriptionSource(os.path.join(rm_multiage_launch_dir,'launch_robot.launch.py')),
        #                 launch_arguments = {
        #                 'namespace': "robot_2",
        #                 'use_namespace': 'True',
        #                 'use_sim_time': use_sim_time,
        #                 'robot_num': "robot_2"}.items()
        #         )
        # robot_3_bringup = IncludeLaunchDescription(
        #                 PythonLaunchDescriptionSource(os.path.join(rm_multiage_launch_dir,'launch_robot.py')),
        #                 launch_arguments = {
        #                 'namespace': "robot_3",
        #                 'use_namespace': 'True',
        #                 'use_sim_time': use_sim_time,
        #                 'robot_num': "robot_3"}.items()
        #         )
        
        start_judgesys = Node(
                package='rm_judgesys',
                executable='judgesys_control_node',
                name='judgesys_control_node',
                output='screen',
                parameters=[battlefile_params]
        )

        start_robot_physics = Node(
                package='rm_multiage',
                executable='robot_physics_node',
                name='robot_physics_node',
                output='screen',
                parameters=[battlefile_params]
        )

        # start_decision_red = Node(
        #         package='rm_decision',
        #         executable='collective_decision',
        #         name='collective_decision',
        #         output='screen',
        #         arguments=[
        #                 '--ros-args',
        #                 '-r', '__ns:=red',
        #                 '-p', 'use_sim_time:=True'
        #         ],
        #         parameters=[battlefile_params]
        #         )

        # start_decision_blue = Node(
        #         package='rm_decision',
        #         executable='collective_decision',
        #         name='collective_decision',
        #         output='screen',
        #         arguments=[
        #                 '--ros-args',
        #                 '-r', '__ns:=blue',
        #                 '-p', 'use_sim_time:=True'
        #         ],
        #         parameters=[battlefile_params]
        #         )

        ld = LaunchDescription()
        # Declare the launch options
        ld.add_action(declare_use_sim_time_cmd)
        ld.add_action(declare_nav_rviz_cmd)
        ld.add_action(robot_0_bringup)
        ld.add_action(robot_1_bringup)
        # ld.add_action(robot_2_bringup)
        # ld.add_action(robot_3_bringup)
        ld.add_action(start_judgesys)
        ld.add_action(start_robot_physics)
        # ld.add_action(start_decision_red)
        # ld.add_action(start_decision_blue)
        return ld
