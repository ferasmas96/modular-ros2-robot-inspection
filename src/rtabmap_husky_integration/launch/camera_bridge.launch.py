from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('rtabmap_husky_integration')) / 'config/camera_bridge.yaml'
    return LaunchDescription([Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='rgbd_camera_bridge',
        namespace='/a300_00000/sensors', output='screen',
        parameters=[{'use_sim_time': True, 'config_file': str(config)}])])
