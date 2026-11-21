#!/usr/bin/python3
"""Scope the reviewed RTAB binary prefix to its own launch, never to GLIM."""
import argparse
import json
import os
from pathlib import Path
import subprocess


def isolated_environment(root, original):
    root = Path(root).resolve()
    if not (root / 'PACKAGE_ARCHIVES.json').is_file():
        raise ValueError('Prefix needs its reviewed package archive manifest')
    ros = root / 'opt/ros/jazzy'
    for package, exe in [('rtabmap_odom', 'rgbd_odometry'), ('rtabmap_slam', 'rtabmap')]:
        if not (ros / 'lib' / package / exe).is_file():
            raise ValueError('Missing native executable: ' + package + '/' + exe)
    env = dict(original)
    env['HUSKY_RTABMAP_ISOLATED_PREFIX'] = str(root)
    paths = {
        'AMENT_PREFIX_PATH': [ros],
        'LD_LIBRARY_PATH': [ros / 'lib/x86_64-linux-gnu', ros / 'lib', root / 'usr/lib/x86_64-linux-gnu'],
        'PYTHONPATH': [ros / 'lib/python3.12/site-packages'],
        'PATH': [ros / 'bin'],
    }
    for key, value in paths.items():
        env[key] = ':'.join(map(str, value)) + (':' + original[key] if original.get(key) else '')
    return env


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--check', action='store_true', help='ELF checks only; no ROS node initialization')
    parser.add_argument('--database', type=Path)
    args = parser.parse_args()
    env = isolated_environment(args.prefix, os.environ)
    ros = args.prefix.resolve() / 'opt/ros/jazzy'
    checks = []
    for target in [ros / 'lib/rtabmap_odom/rgbd_odometry', ros / 'lib/rtabmap_slam/rtabmap']:
        result = subprocess.run(['ldd', '-r', str(target)], env=env, capture_output=True, text=True)
        output = result.stdout + result.stderr
        errors = [line for line in output.splitlines() if 'not found' in line or 'undefined symbol' in line]
        gtsam = [line.strip() for line in output.splitlines() if line.strip().startswith('libgtsam.so')]
        if result.returncode or errors or len(gtsam) != 1 or str(ros / 'lib/x86_64-linux-gnu/libgtsam.so.4') not in gtsam[0]:
            raise RuntimeError('Unsafe native library resolution: ' + str(target) + '\n' + output)
        checks.append({'target': target.name, 'gtsam': gtsam[0].split(' (')[0], 'unresolved': 0})
    if args.check:
        print(json.dumps(checks, indent=2))
        return
    if args.database is None or not args.database.is_absolute():
        parser.error('--database requires a fresh absolute database path')
    run = args.database.parent.parent
    env['ROS_LOG_DIR'] = str(run / 'logs')
    env['ROS_HOME'] = str(run / 'ros_home')
    command = ['ros2', 'launch', 'rtabmap_husky_integration', 'mapping.launch.py',
               'database_path:=' + str(args.database)]
    os.execvpe(command[0], command, env)


if __name__ == '__main__':
    main()
