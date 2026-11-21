#!/usr/bin/env python3
"""Read-only configuration/link audit; never instantiate GLIM or ROS nodes."""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


def read_json(path):
    # GLIM accepts C/C++ comments; preserve quoted strings while removing comments.
    text = re.sub(r'("(?:\\.|[^"\\])*")|//[^\n]*|/\*[\s\S]*?\*/',
                  lambda match: match[1] if match[1] else ' ', path.read_text())
    return json.loads(text)


def differences(left, right, prefix=''):
    if isinstance(left, dict) and isinstance(right, dict):
        result = []
        for key in sorted(left.keys() | right.keys()):
            path = f'{prefix}.{key}'.lstrip('.')
            if key not in left or key not in right:
                result.append(path)
            else:
                result.extend(differences(left[key], right[key], path))
        return result
    return [] if left == right else [prefix]


def validate(baseline, config):
    files = {path.name: read_json(path) for path in config.glob('*.json')}
    originals = {path.name: read_json(path) for path in baseline.glob('*.json')}
    assert len(originals) == 15 and len(files) == 16
    assert files.keys() == originals.keys() | {'config_pose_output.json'}
    expected = {'config.json': ['global.config_pose_output'],
                'config_ros.json': ['glim_ros.extension_modules'],
                'config_logging.json': ['logging.log_dir']}
    changes = {}
    for name, before in originals.items():
        delta = differences(before, files[name])
        assert delta == expected.get(name, []), (name, delta)
        if delta:
            changes[name] = delta
        else:
            assert (baseline / name).read_bytes() == (config / name).read_bytes(), name
    for key, filename in files['config.json']['global'].items():
        if key != 'config_path':
            assert Path(filename).name == filename and filename in files, (key, filename)
    ros = files['config_ros.json']['glim_ros']
    assert ros['enable_local_mapping'] and ros['enable_global_mapping']
    assert ros['extension_modules'] == ['libglim_pose_output.so']
    assert ros['publish_imu2lidar'] is False
    assert ros['imu_topic'] == '/a300_00000/sensors/imu_0/data'
    assert ros['points_topic'] == '/a300_00000/sensors/helios_16p/points_glim'
    assert files['config_sensors.json']['sensors']['T_lidar_imu'] == [0, 0, -.0475, 0, 0, 0, 1]
    assert files['config_pose_output.json']['pose_output']['T_base_imu'] == [0, 0, .294759, 0, 0, 0, 1]
    modules = [files[name][section]['so_name'] for name, section in [
        ('config_odometry_cpu.json', 'odometry_estimation'),
        ('config_sub_mapping_cpu.json', 'sub_mapping'),
        ('config_global_mapping_cpu.json', 'global_mapping')]]
    for module in modules:
        ctypes.CDLL(module, mode=os.RTLD_NOW | os.RTLD_LOCAL)
    return {'configuration': config.name, 'json_files': len(files), 'differences': changes,
            'byte_identical_baseline_files': 12, 'native_cpu_libraries_resolved': modules,
            'log_dir': files['config_logging.json']['logging']['log_dir'], 'result': 'PASS'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--config', type=Path, action='append', required=True)
    parser.add_argument('--library', type=Path, required=True)
    args = parser.parse_args()
    results = [validate(args.baseline, path) for path in args.config]
    library = ctypes.CDLL(str(args.library.resolve()), mode=os.RTLD_NOW | os.RTLD_LOCAL)
    assert getattr(library, 'create_extension_module')  # Resolve only; never call factory.
    symbols = subprocess.check_output(['nm', '-D', str(args.library)], text=True)
    assert not re.search(r'TransformBroadcaster|sendTransform|tf2_ros|tf2_msgs', symbols)
    dependencies = subprocess.check_output(['ldd', str(args.library)], text=True)
    assert 'not found' not in dependencies
    assert not re.search(r'librviz_viewer|libtf2', dependencies)
    print(json.dumps({'result': 'PASS', 'configurations': results,
                      'library_sha256': hashlib.sha256(args.library.read_bytes()).hexdigest(),
                      'factory_resolves': True, 'factory_executed': False,
                      'tf_symbols_or_libraries': False, 'ros_nodes_started': False}, indent=2))


if __name__ == '__main__':
    main()
