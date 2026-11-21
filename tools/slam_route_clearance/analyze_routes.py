#!/usr/bin/env python3
"""Offline geometry inspection only; no ROS initialization or motion publisher."""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from scipy.optimize import LinearConstraint, minimize
import xacro
from ament_index_python.packages import get_package_share_directory


SOURCES = {}


def read(path):
    path = Path(path).resolve()
    data = path.read_bytes()
    SOURCES[str(path)] = hashlib.sha256(data).hexdigest()
    return data


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    result[:3, 3] = xyz
    return result


def sdf_pose(element):
    if element is None:
        return np.eye(4)
    if element.get('relative_to'):
        raise ValueError('Explicit SDF relative_to needs separate frame resolution')
    values = np.fromstring(element.text, sep=' ')
    assert len(values) == 6
    return transform(values[:3], values[3:])


def urdf_pose(element):
    if element is None:
        return np.eye(4)
    return transform(np.fromstring(element.get('xyz', '0 0 0'), sep=' '),
                     np.fromstring(element.get('rpy', '0 0 0'), sep=' '))


def apply(vertices, pose):
    return vertices @ pose[:3, :3].T + pose[:3, 3]


def mesh(path):
    data = read(path)
    if Path(path).suffix.lower() == '.stl':
        count = int.from_bytes(data[80:84], 'little')
        assert len(data) == 84 + count * 50, 'Only binary installed STL supported'
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)),
                          ('attribute', '<u2')])
        triangles = np.frombuffer(data, dtype=dtype, offset=84)['vertices'].astype(float)
        return triangles.reshape(-1, 3), triangles
    if Path(path).suffix.lower() == '.obj':
        vertices, faces = [], []
        for line in data.decode().splitlines():
            tokens = line.split()
            if tokens and tokens[0] == 'v':
                vertices.append([float(v) for v in tokens[1:4]])
            elif tokens and tokens[0] == 'f':
                indices = [int(v.split('/')[0]) for v in tokens[1:]]
                indices = [i - 1 if i > 0 else len(vertices) + i for i in indices]
                faces.extend([[indices[0], indices[i], indices[i + 1]]
                              for i in range(1, len(indices) - 1)])
        vertices = np.array(vertices)
        return vertices, vertices[np.array(faces)]
    raise ValueError('Unsupported collision mesh: ' + str(path))


def box(size):
    return np.array(list(itertools.product(*[(-v / 2, v / 2) for v in size])))


def cylinder(radius, length):
    # Circumscribed 64-sided solid; <=0.2 mm radial excess for these wheels.
    angles = (np.arange(64) + .5) * 2 * math.pi / 64
    radius = radius / math.cos(math.pi / 64)
    return np.array([[radius * math.cos(a), radius * math.sin(a), z]
                     for a in angles for z in [-length / 2, length / 2]])


def hull(points):
    points = np.unique(np.asarray(points), axis=0)
    if len(points) < 3 or np.linalg.matrix_rank(points - points[0]) < 2:
        if len(points) <= 1:
            return points
        _, _, basis = np.linalg.svd(points - points[0])
        order = np.argsort(points @ basis[0])
        return points[order[[0, -1]]]
    return points[ConvexHull(points).vertices]


def point_distances(points, polygon):
    if len(polygon) == 1:
        return np.linalg.norm(points - polygon[0], axis=1)
    edges = np.roll(polygon, -1, axis=0) - polygon
    delta = points[:, None, :] - polygon
    length = np.sum(edges * edges, axis=1)
    t = np.clip(np.sum(delta * edges, axis=2) / length, 0, 1)
    distances = np.linalg.norm(delta - t[:, :, None] * edges, axis=2).min(axis=1)
    if len(polygon) > 2:
        crosses = edges[None, :, 0] * delta[:, :, 1] - edges[None, :, 1] * delta[:, :, 0]
        distances[np.all(crosses >= -1e-10, axis=1)] = 0
    return distances


def polygons_distance(a, b):
    # Convex separation test also covers edge crossings without vertex containment.
    axes = []
    for p in [a, b]:
        if len(p) > 1:
            e = np.roll(p, -1, axis=0) - p
            axes.extend(np.c_[-e[:, 1], e[:, 0]])
    if axes and not any((a @ n).max() < (b @ n).min() - 1e-10 or
                        (b @ n).max() < (a @ n).min() - 1e-10 for n in axes):
        return 0.0
    return float(min(point_distances(a, b).min(), point_distances(b, a).min()))


def support_plane_gap(a, b):
    """Certified convex separation lower bound; solver status alone is not proof."""
    n = b.mean(axis=0) - a.mean(axis=0)
    n /= np.linalg.norm(n)
    initial = np.r_[n, (a @ n).max(), (b @ n).min()]
    constraints = np.concatenate([
        np.c_[-a, np.ones(len(a)), np.zeros(len(a))],
        np.c_[b, np.zeros(len(b)), -np.ones(len(b))]])
    solution = minimize(
        lambda v: v[3] - v[4], initial,
        jac=lambda v: np.array([0., 0, 0, 1, -1]),
        constraints=[LinearConstraint(constraints, 0, np.inf),
                     {'type': 'ineq', 'fun': lambda v: 1 - v[:3] @ v[:3],
                      'jac': lambda v: np.r_[-2 * v[:3], 0, 0]}],
        method='SLSQP', options={'ftol': 1e-11, 'maxiter': 100})
    n = solution.x[:3]
    n /= np.linalg.norm(n)
    # Recompute from actual vertices and a unit normal, independent of optimizer
    # feasibility/tolerances. This gap is always a lower bound on 3D distance.
    gap = float((b @ n).min() - (a @ n).max())
    assert np.isfinite(gap) and solution.success
    return max(0., gap)


def robot_geometry(path):
    read(path)
    # Inspect existing xacro in memory only. Never invoke Clearpath generators.
    document = xacro.process_file(str(path), mappings={'is_sim': 'true',
                                  'namespace': 'a300_00000'})
    root = ET.fromstring(document.toxml())
    for filename in xacro.all_includes:
        read(filename)
    links = {'base_link': np.eye(4)}
    joints = root.findall('joint')
    while len(links) < len(root.findall('link')):
        before = len(links)
        for joint in joints:
            parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
            if parent in links and child not in links:
                assert joint.get('type') in ['fixed', 'continuous']
                links[child] = links[parent] @ urdf_pose(joint.find('origin'))
        assert len(links) > before, 'Unresolved robot kinematics'
    parts = []
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')[0]
            if geometry.tag == 'mesh':
                uri = geometry.get('filename')
                assert uri.startswith('package://')
                package, relative = uri[10:].split('/', 1)
                vertices, _ = mesh(Path(get_package_share_directory(package)) / relative)
                vertices *= np.fromstring(geometry.get('scale', '1 1 1'), sep=' ')
            elif geometry.tag == 'box':
                vertices = box(np.fromstring(geometry.get('size'), sep=' '))
            elif geometry.tag == 'cylinder':
                vertices = cylinder(float(geometry.get('radius')), float(geometry.get('length')))
            else:
                raise ValueError('Unsupported robot geometry ' + geometry.tag)
            vertices = apply(vertices, links[link.get('name')] @ urdf_pose(collision.find('origin')))
            parts.append((link.get('name'), vertices))
    return parts, links


def resolve_uri(uri, directory, fuel):
    if uri.startswith(('https://', 'http://')):
        rest = uri.split('/models/')[1]
        if '/files/' in rest:
            model_version, filename = rest.split('/files/', 1)
            name, version = model_version.rsplit('/', 1)
            matches = list(fuel.glob('*/models/' + name.lower() + '/' + version + '/' + filename))
        else:
            name = rest.rstrip('/').lower()
            matches = list(fuel.glob('*/models/' + name + '/*/model.sdf'))
        assert len(matches) == 1, (uri, matches)
        return matches[0]
    return directory / uri


def world_geometry(path, fuel):
    world = ET.fromstring(read(path)).find('world')
    assert world.get('name') == 'warehouse'
    objects, ground = [], []
    for model in world.findall('model'):
        assert model.get('name') == 'ground_plane'
        assert model.find('.//geometry/plane') is not None
        ground.append(model.get('name'))
    for include in world.findall('include'):
        name = include.findtext('name')
        model_path = resolve_uri(include.findtext('uri').strip(), path.parent, fuel)
        model = ET.fromstring(read(model_path)).find('model')
        static = include.findtext('static', model.findtext('static', 'false')).strip().lower() in ['true', '1']
        assert not model.findall('include') and not model.findall('model')
        model_pose = sdf_pose(include.find('pose'))
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                geometry = collision.find('geometry')[0]
                triangles = None
                if geometry.tag == 'mesh':
                    file = resolve_uri(geometry.findtext('uri').strip(), model_path.parent, fuel)
                    vertices, triangles = mesh(file)
                    scale = np.fromstring(geometry.findtext('scale', '1 1 1'), sep=' ')
                    vertices *= scale
                    triangles *= scale
                elif geometry.tag == 'box':
                    vertices = box(np.fromstring(geometry.findtext('size'), sep=' '))
                elif geometry.tag == 'cylinder':
                    vertices = cylinder(float(geometry.findtext('radius')), float(geometry.findtext('length')))
                else:
                    raise ValueError('Unsupported world collision: ' + geometry.tag)
                pose = model_pose @ sdf_pose(link.find('pose')) @ sdf_pose(collision.find('pose'))
                vertices = apply(vertices, pose)
                if triangles is not None:
                    triangles = apply(triangles.reshape(-1, 3), pose).reshape(-1, 3, 3)
                objects.append({'name': name + '/' + collision.get('name'), 'static': static, 'vertices': vertices,
                                'triangles': triangles})
    return objects, ground


def candidate_poses(name):
    forward = [(x, 0., 0.) for x in np.linspace(0, .3, 61)]
    if name == 'A':
        return forward
    if name == 'B':
        return forward + [(.3, 0., yaw) for yaw in np.linspace(0, .2, 101)[1:]]
    if name == 'C':
        return [(-x, 0., 0.) for x in np.linspace(0, .3, 61)]
    raise ValueError(name)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ['source-run', 'robot-xacro', 'world', 'fuel-root', 'output']:
        parser.add_argument('--' + arg, type=Path, required=True)
    args = parser.parse_args()
    assert not args.output.exists(), 'Refuse to overwrite analysis output'
    identity = json.loads(read(args.source_run / 'experiment_identity.json'))
    reference = json.loads(read(args.source_run / 'stationary_bag_validation.json'))['reference']
    assert np.allclose(identity['live_settled_model_xyz_m'], [0, 0, .13596983616624614], atol=1e-9)
    assert reference['model_base_identity_all_messages'] and reference['max_position_change_m'] == 0
    parts, links = robot_geometry(args.robot_xacro)
    vertices = np.concatenate([v for _, v in parts])
    footprint = hull(vertices[:, :2])
    chassis = hull(dict(parts)['chassis_link'][:, :2])
    objects, ground = world_geometry(args.world, args.fuel_root)
    # Roof surfaces outside the body+margin height cannot constrain horizontal
    # passage. Whole ground-connected objects keep their full projection: this
    # deliberately includes the standing person's upper-body overhang.
    projections = []
    support_triangles = 0
    upper = .13596983616624614 + max(vertices[:, 2].max(), links['helios_16p_link'][2, 3]) + .5 + .05
    for item in objects:
        if item['name'].startswith('warehouse/'):
            for triangle in item['triangles']:
                if triangle[:, 2].max() <= .02:
                    support_triangles += 1
                    continue
                if triangle[:, 2].min() > upper:
                    continue
                projections.append((item['name'], hull(triangle[:, :2])))
        else:
            projections.append((item['name'], hull(item['vertices'][:, :2])))
    # Inspect the bag's complete adapted stream using its native deserializer
    # (no ROS nodes). All data remain in the original private run directory.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.source_run / 'bag'), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    cloud_sets, cloud_stamps, cloud_count = {}, [], 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if not topic.endswith('/helios_16p/points_glim'):
            continue
        msg = deserialize_message(data, PointCloud2)
        assert not msg.is_bigendian and msg.row_step == msg.width * msg.point_step
        values = np.ndarray((msg.width * msg.height, 3), dtype='<f4',
                            buffer=bytes(msg.data), strides=(msg.point_step, 4)).copy()
        key = hashlib.sha256(values.tobytes()).hexdigest()
        if key not in cloud_sets:
            cloud_sets[key] = values[np.isfinite(values).all(axis=1)]
        cloud_count += 1
        cloud_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9)
    points = np.unique(np.concatenate(list(cloud_sets.values())), axis=0).astype(float)
    points += links['helios_16p_link'][:3, 3]
    obstacle_points = points[(points[:, 2] > .01) & (points[:, 2] < 1.0) &
                             (np.linalg.norm(points[:, :2], axis=1) > .7)]
    radius = float(np.linalg.norm(footprint, axis=1).max())
    body_3d = vertices[ConvexHull(vertices).vertices]
    person_3d = next(x['vertices'] for x in objects if x['name'].startswith('Person 1'))
    uncertainty = .05 + 2 * radius * math.sin(.02 / 2) + .01 + .01
    results = {}
    for name in ['A', 'B', 'C']:
        poses = candidate_poses(name)
        best = (math.inf, None, None)
        cloud_best = math.inf
        # Broad phase discards shapes >2 m from the entire candidate region.
        near = []
        for label, polygon in projections:
            low, high = polygon.min(axis=0), polygon.max(axis=0)
            if np.linalg.norm(np.maximum(np.maximum(low - [1, .7], [-1, -.7] - high), 0)) < 2:
                near.append((label, polygon))
        for x, y, yaw in poses:
            R = Rotation.from_euler('z', yaw).as_matrix()[:2, :2]
            body = footprint @ R.T + [x, y]
            for label, polygon in near:
                distance = polygons_distance(body, polygon)
                if distance < best[0]:
                    best = (distance, label, [x, y, yaw])
            cloud_best = min(cloud_best, float(point_distances(obstacle_points[:, :2], body).min()))
        discretization = .0025 + radius * .001
        lower_bound = max(0., best[0] - discretization - uncertainty)
        straight_end = -.3 if name == 'C' else .3
        continuous_translation = hull(np.concatenate([footprint, footprint + [straight_end, 0]]))
        exact_translation = min(polygons_distance(continuous_translation, p) for _, p in projections)
        assert exact_translation <= best[0] + 1e-8 or name == 'B'
        if name != 'B':
            assert best[0] - exact_translation <= .0025 + 1e-8
        person_3d_lower = math.inf
        for x, y, yaw in poses:
            world_body = apply(body_3d, transform([x, y, .13596983616624614], [0, 0, yaw]))
            person_3d_lower = min(person_3d_lower, support_plane_gap(world_body, person_3d))
        results[name] = {'world_projection_minimum_m': best[0], 'limiting_obstacle': best[1],
                         'limiting_pose_xy_yaw': best[2], 'lidar_point_minimum_m': cloud_best,
                         'pose_samples': len(poses), 'sampling_bound_m': discretization,
                         'illustrative_uncertainty_allowance_m': uncertainty,
                         'conservative_lower_bound_m': lower_bound,
                         'exact_continuous_translation_clearance_m': exact_translation,
                         'person_only_3d_sweep_separation_lower_bound_m': person_3d_lower,
                         'person_only_3d_diagnostic_is_full_world_certificate': False,
                         'clearance_pass': lower_bound >= .5,
                         'full_3d_collision_certificate': False}
    stationary_best = min((polygons_distance(footprint, p), label) for label, p in projections)
    result = {'result': 'NO_ROUTE_VERIFIED' if not any(x['clearance_pass'] for x in results.values())
              else 'REQUIRES_SEPARATE_3D_REVIEW', 'required_margin_m': .5,
              'robot_collision_parts': [n for n, _ in parts], 'robot_collision_count': len(parts),
              'robot_xyz_bounds': [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()],
              'whole_robot_footprint_xy': footprint.tolist(), 'chassis_footprint_xy': chassis.tolist(),
              'world_collision_count': len(objects), 'world_model_count': len(set(x['name'].split('/')[0] for x in objects)),
              'nominal_only_dynamic_models': sorted(set(x['name'].split('/')[0] for x in objects if not x['static'])),
              'ground_support_exclusions': ground, 'warehouse_floor_triangles_excluded': support_triangles,
              'stationary_projected_clearance_m': stationary_best[0], 'stationary_limiting_obstacle': stationary_best[1],
              'adapted_bag_clouds_inspected': cloud_count, 'unique_xyz_clouds': len(cloud_sets),
              'bag_cloud_first_stamp': cloud_stamps[0], 'bag_cloud_last_stamp': cloud_stamps[-1],
              'finite_unique_points': len(points), 'candidates': results,
              'source_hashes': SOURCES,
              'limitations': ['World model projections are conservative over occupied heights; upper-body overhang can reduce the planar bound without proving a 3D violation.',
                              'Finite LiDAR returns do not certify unobserved or occluded free space.',
                              'Whole-robot hull fills gaps; wheel cylinders use circumscribed 64-sided bounds.',
                              'No runtime localization/control envelope has been validated; uncertainty allowance is an explicit planning assumption, not measured performance.',
                              'No candidate passes the retained conservative planar margin; no full 3D certificate or runtime reservation is issued.']}
    args.output.mkdir(parents=True)
    with (args.output / 'analysis.json').open('x') as file:
        json.dump(result, file, indent=2)
        file.write('\n')
    np.savez(args.output / 'geometry.npz', footprint=footprint, chassis=chassis,
             person=hull(next(x['vertices'] for x in objects if x['name'].startswith('Person 1'))[:, :2]),
             lidar_obstacles=obstacle_points)
    print(json.dumps({k: result[k] for k in ['result', 'robot_collision_count', 'world_collision_count',
                                           'adapted_bag_clouds_inspected', 'unique_xyz_clouds',
                                           'stationary_projected_clearance_m', 'candidates']}, indent=2))


if __name__ == '__main__':
    main()
