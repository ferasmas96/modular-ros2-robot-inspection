import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('husky_a300_sensor_extensions'),
        'config',
        'rs_helios_16p_bridge.yaml',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation time for the bridge node',
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='rs_helios_16p_bridge',
                parameters=[
                    {
                        'config_file': config_file,
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }
                ],
                output='screen',
            ),
        ]
    )
