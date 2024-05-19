import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  rm_mapping_dir = get_package_share_directory('rm_mapping')

  gmapping_params = os.path.join(rm_mapping_dir, 'config', 'gmapping_params.yaml')

  gmapping_node = Node(
    package='gmapping',
    executable='slam_gmapping',
    name='slam_gmapping',
    output='screen',
    parameters=[gmapping_params],
    remappings=[('scan', '/scan')],
  )

  return LaunchDescription([gmapping_node])
