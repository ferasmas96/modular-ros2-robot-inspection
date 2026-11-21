from glob import glob
from setuptools import setup

name = 'rtabmap_husky_integration'
setup(
    name=name, version='0.1.0', packages=[name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + name]),
        ('share/' + name, ['package.xml']),
        ('share/' + name + '/launch', glob('launch/*.launch.py')),
        ('share/' + name + '/config', glob('config/*.yaml')),
        ('share/' + name + '/urdf', glob('urdf/*.xacro')),
    ],
    entry_points={'console_scripts': ['pose_adapter = ' + name + '.pose_adapter:main']},
    license='MIT', description='TF-free independent RGB-D SLAM for Husky.',
)
