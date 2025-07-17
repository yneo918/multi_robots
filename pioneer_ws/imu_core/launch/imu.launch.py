import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions

def generate_launch_description():
    ld = launch.LaunchDescription()
    
    # Get robot ID from environment
    robot_id = os.getenv("ROBOT_ID", "robot1")
    
    # Use system config - check if config file exists
    config_path = os.path.join(
        get_package_share_directory('imu_core'),
        'config',
        'imu_params.yaml'
    )
    
    # Check if config file exists, if not launch without parameters
    parameters = []
    if os.path.exists(config_path):
        parameters = [config_path]
    
    node = launch_ros.actions.Node(
        package='imu_core',
        name=f'{robot_id}_imu',
        executable='run_imu',
        parameters=parameters,
        output='screen'
    )
    ld.add_action(node)
    return ld