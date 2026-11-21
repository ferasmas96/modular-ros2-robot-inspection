#!/usr/bin/python3
"""Static camera/TF/preservation validation, no ROS or Gazebo runtime."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import xacro
import yaml


def validate(baseline, setup, repository, sdf):
    spec = importlib.util.spec_from_file_location('route_geometry', repository / 'tools/slam_route_clearance/analyze_routes.py')
    g = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(g)
    old_parts, old_links = g.robot_geometry(baseline / 'robot.urdf.xacro')
    parts, links = g.robot_geometry(setup / 'robot.urdf.xacro')
    for name, vertices in old_parts:
        assert np.array_equal(vertices, dict(parts)[name]), name
    for name, pose in old_links.items():
        assert np.array_equal(pose, links[name]), name
    assert len(parts) == len(old_parts) + 2
    xyz = links['camera_0_bottom_screw_frame'][:3, 3]
    assert np.allclose(xyz, [.35, .18, .35], atol=1e-12)
    optical = links['camera_0_color_optical_frame']
    assert np.allclose(optical[:3, 3], [.3606, .2125, .3625], atol=1e-12)
    assert np.allclose(optical[:3, :3], [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], atol=1e-12)
    old_xml = ET.fromstring(xacro.process_file(str(baseline / 'robot.urdf.xacro'), mappings={'is_sim': 'true', 'namespace': 'a300_00000'}).toxml())
    new_xml = ET.fromstring(xacro.process_file(str(setup / 'robot.urdf.xacro'), mappings={'is_sim': 'true', 'namespace': 'a300_00000'}).toxml())
    sensors = {s.get('name'): s for s in new_xml.findall('.//sensor')}
    for sensor in old_xml.findall('.//sensor'):
        assert ET.tostring(sensor) == ET.tostring(sensors[sensor.get('name')])
    parsed = ET.parse(sdf).getroot()
    camera = parsed.find(".//sensor[@name='camera_0_rgbd']")
    assert camera is not None and camera.get('type') == 'rgbd_camera'
    assert np.allclose(np.fromstring(camera.findtext('pose'), sep=' '), [.3606, .2125, .3625, 0, 0, 0])
    assert camera.findtext('camera/optical_frame_id') == 'camera_0_color_optical_frame'
    assert camera.findtext('camera/image/width') == '640'
    assert camera.findtext('camera/image/height') == '480'
    config = repository / 'src/rtabmap_husky_integration/config'
    params = yaml.safe_load((config / 'pipeline.yaml').read_text())
    for group in ['visual_odometry', 'mapping']:
        p = params[group]
        assert p['use_sim_time'] is True and p['publish_tf'] is False
        assert p['frame_id'] == 'base_link' and p['approx_sync'] is False
    assert params['visual_odometry']['odom_frame_id'] == 'rtabmap_odom'
    mapper = params['mapping']
    assert mapper['odom_frame_id'] == '' and mapper['subscribe_odom'] is True
    assert mapper['odom_sensor_sync'] is False and mapper['wait_for_transform'] == 0.0
    assert not mapper['subscribe_scan'] and not mapper['subscribe_scan_cloud']
    assert mapper['Grid/Sensor'] == '1' and mapper['Mem/DepthCompressionFormat'] == '.png'
    bridge = yaml.safe_load((config / 'camera_bridge.yaml').read_text())
    assert len(bridge) == 3 and all(v['direction'] == 'GZ_TO_ROS' for v in bridge)
    assert all(v['qos_profile'] == 'SENSOR_DATA' for v in bridge)
    return {'result': 'PASS', 'old_collision_parts_identical': len(old_parts), 'new_collision_parts': len(parts),
            'old_link_transforms_identical': len(old_links), 'baseline_sensor_xml_identical': len(old_xml.findall('.//sensor')),
            'camera_bottom_screw_xyz_m': xyz.tolist(), 'color_optical_T_base': optical.tolist(),
            'sdf_sensor_pose': camera.findtext('pose'), 'typed_parameters_and_bridges': 'PASS',
            'intrinsics': '640x480, hfov=1.25 rad; predicted fx=fy=443.530008 px, verify actual CameraInfo at runtime',
            'registration': 'Native single RGB-D viewpoint; runtime cloud/image check pending',
            'tf': 'No estimator broadcasting; sensor joints only; runtime authority check pending'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ['baseline', 'setup', 'repository', 'sdf', 'output']:
        p.add_argument('--' + name, type=Path, required=True)
    args = p.parse_args()
    result = validate(args.baseline, args.setup, args.repository, args.sdf)
    with args.output.open('x') as f:
        json.dump(result, f, indent=2)
        f.write('\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
