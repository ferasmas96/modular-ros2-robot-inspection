"""Start only native RGB-D odometry/mapping and a TF-free pose adapter."""
from pathlib import Path
import os
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_nodes(context):
    private = os.environ.get('HUSKY_RTABMAP_ISOLATED_PREFIX')
    if not private or Path(get_package_prefix('rtabmap_odom')) != Path(private) / 'opt/ros/jazzy':
        raise RuntimeError('Use tools/rtabmap_isolated/run_isolated.py; native GTSAM versions must stay isolated')
    database = Path(LaunchConfiguration('database_path').perform(context))
    if not database.is_absolute() or not database.parent.is_dir():
        raise RuntimeError('database_path needs an existing private parent and absolute filename')
    if list(database.parent.glob(database.name + '*')):
        raise RuntimeError('Refusing existing database or sidecars: ' + str(database))
    if not os.access(database.parent, os.W_OK):
        raise RuntimeError('Database parent is not writable')
    config = Path(get_package_share_directory('rtabmap_husky_integration')) / 'config/pipeline.yaml'
    params = yaml.safe_load(config.read_text())
    namespace = '/a300_00000/slam/rtabmap'
    tf = [('/tf', '/a300_00000/tf'), ('/tf_static', '/a300_00000/tf_static')]
    images = [('rgb/image', '/a300_00000/sensors/camera_0/color/image'),
              ('depth/image', '/a300_00000/sensors/camera_0/depth/image'),
              ('rgb/camera_info', '/a300_00000/sensors/camera_0/color/camera_info')]
    odom = [('odom', namespace + '/odom')]
    return [
        Node(package='rtabmap_odom', executable='rgbd_odometry', name='rgbd_odometry',
             namespace=namespace, parameters=[params['visual_odometry']],
             remappings=tf + images + odom, output='screen'),
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap',
             namespace=namespace, parameters=[params['mapping'], {'database_path': str(database)}],
             remappings=tf + images + odom, output='screen'),
        Node(package='rtabmap_husky_integration', executable='pose_adapter', name='pose_adapter',
             namespace=namespace, parameters=[{'use_sim_time': True}],
             remappings=odom, output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('database_path'), OpaqueFunction(function=make_nodes)])
