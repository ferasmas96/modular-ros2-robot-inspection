#!/usr/bin/env python3
"""SLAM-003D-R2 offline inspection; imports the preserved R geometry routines."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import minimize_scalar
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

import analyze_routes as geometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['source-run', 'robot-xacro', 'world', 'fuel-root', 'output']:
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    assert not args.output.exists(), 'Refuse to overwrite results'
    identity = json.loads(geometry.read(args.source_run / 'experiment_identity.json'))
    reference = json.loads(geometry.read(args.source_run / 'stationary_bag_validation.json'))['reference']
    assert reference['model_base_identity_all_messages'] and reference['max_position_change_m'] == 0
    spawn = np.array(identity['live_settled_model_xyz_m'], dtype=float)
    assert np.allclose(spawn, [0, 0, .13596983616624614], atol=1e-9)
    parts, links = geometry.robot_geometry(args.robot_xacro)
    vertices = np.concatenate([v for _, v in parts])
    footprint = geometry.hull(vertices[:, :2])
    body_3d = vertices[ConvexHull(vertices).vertices]
    radius = float(np.linalg.norm(footprint, axis=1).max())
    objects, ground = geometry.world_geometry(args.world, args.fuel_root)
    projections = []
    floor_exclusions = 0
    upper = spawn[2] + max(vertices[:, 2].max(), links['helios_16p_link'][2, 3]) + .55
    for item in objects:
        if item['name'].startswith('warehouse/'):
            for triangle in item['triangles']:
                if triangle[:, 2].max() <= .02:
                    floor_exclusions += 1
                    continue
                if triangle[:, 2].min() > upper:
                    continue
                projections.append((item['name'], geometry.hull(triangle[:, :2])))
        else:
            projections.append((item['name'], geometry.hull(item['vertices'][:, :2])))
    # Every tested center is in [0,.5]^2; this radius box encloses the entire
    # continuous rotation/translation footprint, not only sampled poses.
    low, high = spawn[:2] - radius, spawn[:2] + .5 + radius
    near = []
    omitted_aabb_bounds = []
    for label, polygon in projections:
        bound = float(np.linalg.norm(np.maximum(np.maximum(polygon.min(0) - high,
                                                           low - polygon.max(0)), 0)))
        if bound < 2:
            near.append((label, polygon))
        else:
            omitted_aabb_bounds.append(bound)
    assert near

    def body_at(yaw, distance=0):
        rotation = Rotation.from_euler('z', yaw).as_matrix()[:2, :2]
        center = spawn[:2] + distance * np.array([math.cos(yaw), math.sin(yaw)])
        return footprint @ rotation.T + center

    def clearance(yaw, distance=0):
        body = body_at(yaw, distance)
        return min((geometry.polygons_distance(body, polygon), label)
                   for label, polygon in near)

    angles = np.linspace(0, 1.57, 1571)
    rotation_rows = [(float(yaw), *clearance(float(yaw))) for yaw in angles]
    radius_bound = 2 * radius * math.sin(.001 / 4)

    def rotation_result(theta):
        candidates = [row for row in rotation_rows if row[0] <= theta]
        candidates.append((theta, *clearance(theta)))
        best = min(candidates, key=lambda row: row[1])
        lo, hi = max(0., best[0] - .001), min(theta, best[0] + .001)
        if hi > lo:
            refine = minimize_scalar(lambda yaw: clearance(yaw)[0], bounds=(lo, hi),
                                     method='bounded', options={'xatol': 1e-12})
            value, label = clearance(float(refine.x))
            if value < best[1]:
                best = (float(refine.x), value, label)
        return {'minimum_projected_m': best[1], 'limiting_obstacle': best[2],
                'limiting_yaw_rad': best[0], 'center_xy_m': spawn[:2].tolist(),
                'sample_count': len(candidates) - 1,
                'continuous_lower_bound_m': max(0., min(row[1] for row in candidates) - radius_bound),
                'between_sample_bound_m': radius_bound}

    def translation_result(theta):
        start, end = body_at(theta), body_at(theta, .5)
        sweep = geometry.hull(np.concatenate([start, end]))
        distance, label, polygon = min(
            ((geometry.polygons_distance(sweep, p), label, p) for label, p in near),
            key=lambda row: row[0])
        # For a fixed convex obstacle the distance along straight translation is
        # convex. Locate a witness and check it against the exact swept polygon.
        objective = lambda s: geometry.polygons_distance(body_at(theta, s), polygon)
        locate = minimize_scalar(objective, bounds=(0, .5), method='bounded',
                                 options={'xatol': 1e-12})
        s, value = min([(0., objective(0)), (.5, objective(.5)),
                        (float(locate.x), float(locate.fun))], key=lambda row: row[1])
        assert abs(value - distance) < 1e-7
        center = spawn[:2] + s * np.array([math.cos(theta), math.sin(theta)])
        return {'minimum_projected_m': distance, 'limiting_obstacle': label,
                'limiting_distance_along_path_m': s, 'limiting_center_xy_m': center.tolist(),
                'heading_rad': theta, 'exact_continuous_sweep': True,
                'end_center_xy_m': (spawn[:2] + .5 * np.array([math.cos(theta), math.sin(theta)])).tolist()}

    uncertainty = .05 + 2 * radius * math.sin(.02 / 2) + .01 + .01
    routes = []
    for theta in [math.radians(x) for x in [5, 10, 15, 30, 45, 60]] + [1.57]:
        rotation = rotation_result(theta)
        translation = translation_result(theta)
        minimum = min(rotation['minimum_projected_m'], translation['minimum_projected_m'])
        bound = min(rotation['continuous_lower_bound_m'], translation['minimum_projected_m'])
        routes.append({'turn_rad': theta, 'turn_degrees': math.degrees(theta),
                       'rotation_seconds_at_0_10_rad_s': theta / .1,
                       'forward_distance_m': .5, 'forward_seconds_at_0_10_m_s': 5,
                       'rotation': rotation, 'translation': translation,
                       'minimum_projected_m': minimum,
                       'continuous_lower_bound_m': bound,
                       'uncertainty_adjusted_lower_bound_m': max(0., bound - uncertainty),
                       'clearance_pass': minimum >= .5 and bound - uncertainty >= .5})

    # Separate person-only 3D diagnostic. It is never used to declare a route safe
    # and is explicitly not an actual full-world continuous 3D clearance result.
    person = next(x['vertices'] for x in objects if x['name'].startswith('Person 1'))
    diagnostic = {'rotation': (math.inf, None), 'translation': (math.inf, None)}
    for phase, samples in [('rotation', [(yaw, 0.) for yaw in np.linspace(0, 1.57, 158)]),
                           ('translation', [(1.57, s) for s in np.linspace(0, .5, 101)])]:
        for yaw, distance in samples:
            center = spawn + [distance * math.cos(yaw), distance * math.sin(yaw), 0]
            body = geometry.apply(body_3d, geometry.transform(center, [0, 0, yaw]))
            gap = geometry.support_plane_gap(body, person)
            if gap < diagnostic[phase][0]:
                diagnostic[phase] = (gap, {'yaw_rad': float(yaw), 'distance_m': float(distance),
                                          'center_xyz_m': center.tolist()})
    start_gap, start_label = clearance(0)
    result = {'task': 'SLAM-003D-R2', 'decision': 'NO_ROUTE_VERIFIED',
              'spawn_world_base_xyz_m': spawn.tolist(), 'spawn_yaw_rad': 0.,
              'robot_collision_count': len(parts), 'world_collision_count': len(objects),
              'world_model_count': len(set(x['name'].split('/')[0] for x in objects)),
              'whole_robot_footprint_xy': footprint.tolist(),
              'initial_projected_clearance_m': start_gap, 'initial_limiting_obstacle': start_label,
              'required_clearance_m': .5, 'footprint_radius_m': radius,
              'uncertainty_allowance_m': uncertainty, 'rotation_sample_step_rad': .001,
              'ground_support_excluded': ground, 'warehouse_floor_triangles_excluded': floor_exclusions,
              'broad_phase_omitted_polygon_count': len(omitted_aabb_bounds),
              'omitted_polygons_minimum_aabb_distance_m': min(omitted_aabb_bounds),
              'routes': routes, 'person_only_3d_sampled_separation_diagnostic': diagnostic,
              'actual_full_world_continuous_3d_clearance_m': None,
              'actual_full_world_continuous_3d_status': 'UNVERIFIED',
              'all_smaller_turns_projection_argument': 'Every candidate includes yaw=0 at the same spawn. The initial projected margin is below 0.5 m, so no smaller positive turn can satisfy the retained conservative projected criterion over its entire route.',
              'source_hashes': geometry.SOURCES,
              'nominal_dynamic_model_poses_unverified': sorted(set(x['name'].split('/')[0] for x in objects if not x['static'])),
              'runtime_executed': False, 'new_experiment_reservation': False}
    assert all(not row['clearance_pass'] for row in routes)
    args.output.mkdir()
    with (args.output / 'analysis.json').open('x') as file:
        json.dump(result, file, indent=2)
        file.write('\n')
    with (args.output / 'rotation_samples.json').open('x') as file:
        json.dump(rotation_rows, file)
        file.write('\n')
    print(json.dumps({'decision': result['decision'], 'initial_projected_clearance_m': start_gap,
                      'main_route': routes[-1], 'smaller_routes': [
                          {'degrees': row['turn_degrees'], 'minimum_projected_m': row['minimum_projected_m'],
                           'pass': row['clearance_pass']} for row in routes[:-1]],
                      'person_only_3d_diagnostic': diagnostic}, indent=2))


if __name__ == '__main__':
    main()
