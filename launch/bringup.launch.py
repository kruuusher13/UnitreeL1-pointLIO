import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate Config
    # We assume you mount/copy your config. 
    # For devcontainer, we point directly to the file we just made.
    config_path = os.path.join(os.getcwd(), 'config', 'unitree_l1.yaml')

    return LaunchDescription([
        # 2. Foxglove Bridge (Visualizer)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'port': 8765, 'send_buffer_limit': 10000000}],
            output='screen'
        ),

        # 3. Point-LIO
        Node(
            package='point_lio',
            executable='pointlio_mapping',
            name='point_lio_mapping',
            output='screen',
            parameters=[config_path],
            # Remapping ensures we don't change source code
            remappings=[
                ('/livox/lidar', '/utlidar/cloud'),
                ('/livox/imu', '/utlidar/imu')
            ]
        )
    ])