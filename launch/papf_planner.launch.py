import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Get the path to your package's share directory
    pkg_share = get_package_share_directory('papf_planner')

    # Get the path to your YAML file
    params_file_path = os.path.join(pkg_share, 'config', 'papf_params.yaml')
    print("--- DEBUG: PARAM FILE PATH ---")
    print(params_file_path)
    print("------------------------------")

    # Launch the papf_node
    papf_node = Node(
        package='papf_planner',
        executable='papf_node', # <-- Make sure this matches your executable name in CMakeLists.txt
        name='papf_node',
        output='screen',
        parameters=[params_file_path] # <-- This line loads your YAML file
    )

    return LaunchDescription([
        papf_node
    ])