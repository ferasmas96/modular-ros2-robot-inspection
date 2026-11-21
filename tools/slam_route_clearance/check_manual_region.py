#!/usr/bin/env python3
"""Offline full-body XY disk bound for the small SLAM-004C operator region."""
import argparse
import json
from pathlib import Path
import numpy as np
import analyze_routes as geometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['robot-xacro', 'world', 'fuel-root', 'output']:
        parser.add_argument('--' + name, type=Path, required=True)
    args = parser.parse_args()
    assert not args.output.exists()
    parts, links = geometry.robot_geometry(args.robot_xacro)
    vertices = np.concatenate([v for _, v in parts])
    radius = float(np.linalg.norm(vertices[:, :2], axis=1).max())
    objects, _ = geometry.world_geometry(args.world, args.fuel_root)
    upper = (.13596983616624614 + max(vertices[:, 2].max(), links['helios_16p_link'][2, 3])
             + .5 + .15)
    center = np.array([[-4., 3.]])
    values = []
    floor = roof = 0
    for item in objects:
        if item['name'].startswith('warehouse/'):
            polygons = []
            for triangle in item['triangles']:
                if triangle[:, 2].max() <= .02:
                    floor += 1
                elif triangle[:, 2].min() > upper:
                    roof += 1
                else:
                    polygons.append(geometry.hull(triangle[:, :2]))
        else:
            polygons = [geometry.hull(item['vertices'][:, :2])]
        for polygon in polygons:
            distance = float(geometry.point_distances(center, polygon)[0])
            values.append((distance - radius - 1.25 - .15, item['name']))
    lower, obstacle = min(values)
    result = {
        'task': 'SLAM-004C', 'collision_parts': len(parts),
        'world_collision_geometries': len(objects), 'full_body_xy_radius_m': radius,
        'center_world_xy_m': [-4., 3.], 'center_radius_limit_m': 1.25,
        'additional_illustrative_allowance_m': .15,
        'minimum_nominal_projected_lower_bound_m': lower, 'limiting_obstacle': obstacle,
        'required_obstacle_margin_m': .5, 'projection_result': 'PASS' if lower >= .5 else 'FAIL',
        'floor_triangles_excluded': floor, 'overhead_triangles_excluded': roof,
        'vertical_cutoff_world_z_m': float(upper),
        'nonstatic_world_collision_objects': sorted(set(o['name'] for o in objects if not o['static'])),
        'source_sha256': geometry.SOURCES,
        'limitations': [
            'Nominal world geometry; dynamic objects require live/visual verification.',
            'Circumscribed full-body XY disk bounds all yaw angles within the center radius.',
            'Ground support/subfloor and sufficiently high roof triangles excluded.',
            'Not a full 3D or measured tracking-error certificate; no runtime or movement.'],
    }
    with args.output.open('x') as out:
        json.dump(result, out, indent=2)
        out.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'source_sha256'}, indent=2))


if __name__ == '__main__':
    main()
