import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    
    # Get the directory of this launch file
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(launch_file_dir)
    # Construct paths to the parameter files relative to the launch file directory
    cluster_file = os.path.join(parent_dir, 'config', 'cluster_multi.yaml')

    # Check if parameter files exist
    if not os.path.isfile(cluster_file):
        raise FileNotFoundError(f"Parameter file not found: {cluster_file}")

    # Nodes
    run_cluster_node = Node(
        package="cluster_node",
        executable="cluster_node",
        parameters=[cluster_file],
    )

    ld.add_action(run_cluster_node)

    return ld