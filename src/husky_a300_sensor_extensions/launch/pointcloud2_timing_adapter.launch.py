from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'input_topic',
                default_value='/a300_00000/sensors/helios_16p/points',
                description='Source PointCloud2 topic',
            ),
            DeclareLaunchArgument(
                'output_topic',
                default_value='/a300_00000/sensors/helios_16p/points_glim',
                description='Separate adapted PointCloud2 topic',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation time',
            ),
            Node(
                package='husky_a300_sensor_extensions',
                executable='pointcloud2_timing_adapter.py',
                name='pointcloud2_timing_adapter',
                parameters=[
                    {
                        'input_topic': LaunchConfiguration('input_topic'),
                        'output_topic': LaunchConfiguration('output_topic'),
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }
                ],
                output='screen',
            ),
        ]
    )
