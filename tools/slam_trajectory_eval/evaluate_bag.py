#!/usr/bin/env python3
"""Read a closed native ROS bag; compare GLIM/base with composed Gazebo/base poses."""
import argparse
import collections
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation

from trajectory_math import (transform, interpolate_reference, stationary_alignment,
                             absolute_errors, relative_errors, statistics)

NS = '/a300_00000'
POSE = NS + '/slam/glim/pose'
REF = NS + '/reference/gazebo_pose'


def stamp(value):
    return value.sec * 10**9 + value.nanosec


def vector(p):
    return [p.x, p.y, p.z]


def quaternion(q):
    return [q.x, q.y, q.z, q.w]


def from_transform(t):
    return transform(vector(t.translation), quaternion(t.rotation))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    metadata = reader.get_metadata()
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    counts, times, frames = collections.Counter(), collections.defaultdict(list), set()
    estimates, references, raw_quaternions = [], [], []
    commands = collections.defaultdict(list)
    wheel = []
    model_base_nonidentity = 0
    guard = []
    while reader.has_next():
        topic, data, receipt = reader.read_next()
        msg = deserialize_message(data, types[topic])
        counts[topic] += 1
        if topic == POSE:
            times[topic].append(stamp(msg.header.stamp))
            frames.add(msg.header.frame_id)
            raw_quaternions.append(quaternion(msg.pose.orientation))
            estimates.append(transform(vector(msg.pose.position), quaternion(msg.pose.orientation)))
        elif topic == REF:
            tree = {(t.header.frame_id, t.child_frame_id): t for t in msg.transforms}
            model = tree.get(('warehouse', 'a300_00000/robot'))
            base = tree.get(('a300_00000/robot', 'a300_00000/robot/base_link'))
            if model is None or base is None or stamp(model.header.stamp) != stamp(base.header.stamp):
                raise ValueError('reference model/base identity or same-stamp composition missing')
            times[topic].append(stamp(model.header.stamp))
            T_model_base = from_transform(base.transform)
            model_base_nonidentity += not np.allclose(T_model_base, np.eye(4), atol=1e-9)
            references.append(from_transform(model.transform) @ T_model_base)
        elif topic.endswith('cmd_vel') or topic.endswith('limited_preview'):
            t = msg.twist
            values = vector(t.linear) + vector(t.angular)
            if not np.isfinite(values).all():
                raise ValueError('nonfinite command')
            commands[topic].append((stamp(msg.header.stamp), receipt, values))
        elif topic in [NS + '/platform/odom', NS + '/platform/odom/filtered']:
            times[topic].append(stamp(msg.header.stamp))
            if topic.endswith('/filtered'):
                wheel.append((stamp(msg.header.stamp), vector(msg.pose.pose.position),
                              vector(msg.twist.twist.linear), vector(msg.twist.twist.angular)))
        elif topic == '/clock':
            times[topic].append(stamp(msg.clock))
        elif topic.endswith('/guard_status'):
            guard.append(json.loads(msg.data))
    assert sum(counts.values()) == metadata.message_count
    assert all(row.message_count == counts[row.topic_metadata.name]
               for row in metadata.topics_with_message_count)
    assert frames == {'glim_odom'} and len(estimates) > 20 and len(references) > 20
    ep = np.asarray(estimates)
    rp = np.asarray(references)
    et = np.asarray(times[POSE], dtype=np.int64) / 1e9
    rt = np.asarray(times[REF], dtype=np.int64) / 1e9
    assert np.all(np.diff(et) > 0) and np.all(np.diff(rt) > 0)
    index, reference, brackets = interpolate_reference(rt, rp, et, max_gap=.10)
    assert len(index) > 20
    time = et[index]
    estimated = ep[index]
    main_commands = commands[NS + '/platform/cmd_vel']
    moving = [row for row in main_commands if max(abs(row[2][0]), abs(row[2][5])) > .005]
    if not moving:
        raise ValueError('no actual robot commands; moving comparison unavailable')
    motion_start, motion_end = moving[0][0] / 1e9, moving[-1][0] / 1e9
    initial_end = min(time[0] + 10, motion_start - .25)
    hold = time <= initial_end
    if hold.sum() < 20 or time[hold][-1] - time[hold][0] < 9.8:
        raise ValueError('ten-second recorded initial stationary alignment hold unavailable')
    initial_ref_displacement = np.linalg.norm(reference[hold, :3, 3] - reference[hold][0, :3, 3], axis=1)
    assert initial_ref_displacement.max() < .01
    hold_wheel = [row for row in wheel if time[0] <= row[0] / 1e9 <= initial_end]
    assert hold_wheel and max(np.linalg.norm(row[2]) for row in hold_wheel) < .01
    alignment = stationary_alignment(estimated[hold], reference[hold])
    aligned = alignment[None] @ estimated
    distance, angle = absolute_errors(aligned, reference)
    active = (time >= motion_start) & (time <= motion_end)
    assert active.sum() > 20
    pairs, rpe_distance, rpe_angle = relative_errors(time, aligned, reference)
    active_pairs = (time[pairs[:, 0]] >= motion_start) & (time[pairs[:, 1]] <= motion_end)
    assert active_pairs.sum() >= 20
    dpos = np.linalg.norm(np.diff(ep[:, :3, 3], axis=0), axis=1)
    drot = Rotation.from_matrix(ep[:-1, :3, :3].transpose(0, 2, 1) @ ep[1:, :3, :3]).magnitude()
    dt = np.diff(et)
    command_checks = {}
    for topic, rows in commands.items():
        data = np.asarray([r[2] for r in rows])
        command_checks[topic] = {'messages': len(rows), 'max_abs_components': np.abs(data).max(0).tolist(),
                                 'zero_header_stamps': sum(r[0] == 0 for r in rows)}
        if topic != NS + '/manual/raw_cmd_vel':
            command_checks[topic]['within_limits'] = bool(np.all(np.abs(data[:, 0]) <= .1 + 1e-9)
                                                         and np.all(np.abs(data[:, 5]) <= .1 + 1e-9)
                                                         and np.all(np.abs(data[:, 1:5]) <= 1e-9))
    norms = np.linalg.norm(raw_quaternions, axis=1)
    last_turn = max((row[0] / 1e9 for row in main_commands if abs(row[2][5]) > .005), default=None)
    result = {
        'status': 'METRICS_VALID_FOR_CAPTURED_INTERVAL',
        'topic_counts': dict(counts), 'metadata_message_count': metadata.message_count,
        'live_pose': {'messages': len(ep), 'timestamp_range_s': [float(et[0]), float(et[-1])],
                      'frequency_sensor_hz': float((len(ep)-1)/(et[-1]-et[0])),
                      'strictly_increasing': True, 'frame': 'glim_odom', 'body': 'base_link',
                      'position_min': ep[:, :3, 3].min(0).tolist(), 'position_max': ep[:, :3, 3].max(0).tolist(),
                      'quaternion_norm_range': [float(norms.min()), float(norms.max())],
                      'max_step_translation_m': float(dpos.max()), 'max_step_rotation_rad': float(drot.max()),
                      'large_jump_count': int(np.sum((dpos > .1*dt+.05) | (drot > .1*dt+.05))),
                      'messages_after_last_turn': int(np.sum(et > last_turn)) if last_turn else None},
        'reference': {'messages': len(rp), 'frame': 'warehouse', 'body': 'base_link',
                      'composition': 'T_world_model * T_model_base at identical sensor stamps',
                      'nonidentity_model_base_messages': int(model_base_nonidentity),
                      'path_length_m': float(np.linalg.norm(np.diff(rp[:, :3, 3], axis=0), axis=1).sum()),
                      'max_center_distance_from_spawn_m': float(np.linalg.norm(rp[:, :2, 3]-[-4, 3], axis=1).max()),
                      'net_displacement_m': float(np.linalg.norm(rp[-1, :3, 3]-rp[0, :3, 3])),
                      'timestamp_range_s': [float(rt[0]), float(rt[-1])]},
        'synchronization': {'matched_pairs': len(index), 'excluded_without_extrapolation': len(ep)-len(index),
                            'coverage_fraction': len(index)/len(ep),
                            'max_reference_bracket_s': float(brackets.max()),
                            'method': 'header timestamps; linear translation + quaternion SLERP; no extrapolation'},
        'alignment': {'method': 'one fixed metric SE(3), initial recorded stationary hold; no scale/whole-route fit',
                       'matrix_T_world_glim': alignment.tolist(), 'pairs': int(hold.sum()),
                       'time_range_s': [float(time[hold][0]), float(time[hold][-1])],
                       'reference_stationary_max_displacement_m': float(initial_ref_displacement.max())},
        'actual_command_motion_interval_s': [motion_start, motion_end],
        'ate_translation_m_all': statistics(distance), 'ate_orientation_deg_all': statistics(np.rad2deg(angle)),
        'ate_translation_m_motion_interval': statistics(distance[active]),
        'ate_orientation_deg_motion_interval': statistics(np.rad2deg(angle[active])),
        'rpe_1s_translation_m_all': statistics(rpe_distance),
        'rpe_1s_rotation_deg_all': statistics(np.rad2deg(rpe_angle)),
        'rpe_1s_translation_m_motion_interval': statistics(rpe_distance[active_pairs]),
        'rpe_1s_rotation_deg_motion_interval': statistics(np.rad2deg(rpe_angle[active_pairs])),
        'rpe_interval_s': 1., 'rpe_pair_tolerance_s': .03,
        'command_checks': command_checks,
        'wheel_max_linear_speed_m_s': float(max(np.linalg.norm(row[2]) for row in wheel)),
        'wheel_max_angular_speed_rad_s': float(max(np.linalg.norm(row[3]) for row in wheel)),
        'stream_gaps': {topic: {'non_increasing': int(np.sum(np.diff(np.asarray(t, dtype=np.int64)) <= 0)),
                                'max_gap_s': float(np.diff(np.asarray(t, dtype=np.int64)).max()/1e9)}
                        for topic, t in times.items() if len(t) > 1},
        'bag_file_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in args.bag.iterdir() if p.is_file()},
        'limitations': ['Captured interval only; experiment outcome/safety abort reported separately.',
                       'Synthetic simulation/Mode A timing; no general accuracy claim.',
                       'Wheel odometry checks motion, not independent ground truth.',
                       'Header gaps and logger loss notices are distinct; no exact zero-loss claim.'],
    }
    with (args.output/'metrics.json').open('x') as out:
        json.dump(result, out, indent=2)
        out.write('\n')
    with (args.output/'aligned_poses.csv').open('x') as out:
        writer = csv.writer(out)
        writer.writerow(['simulation_seconds', 'ref_x_m', 'ref_y_m', 'ref_z_m',
                         'glim_x_m', 'glim_y_m', 'glim_z_m', 'translation_error_m',
                         'orientation_error_rad', 'motion_interval'])
        for i, stamp_s in enumerate(time):
            writer.writerow([stamp_s, *reference[i, :3, 3], *aligned[i, :3, 3],
                             distance[i], angle[i], int(active[i])])
    with (args.output/'commands.csv').open('x') as out:
        writer = csv.writer(out)
        writer.writerow(['topic', 'sensor_seconds', 'bag_receipt_seconds', 'linear_x_m_s', 'angular_z_rad_s'])
        for topic, rows in commands.items():
            for ts, receipt, values in rows:
                writer.writerow([topic, ts/1e9, receipt/1e9, values[0], values[5]])
    print(json.dumps({k: result[k] for k in ['status', 'live_pose', 'reference', 'synchronization',
                     'actual_command_motion_interval_s', 'ate_translation_m_motion_interval',
                     'rpe_1s_translation_m_motion_interval', 'rpe_1s_rotation_deg_motion_interval',
                     'command_checks']}, indent=2))


if __name__ == '__main__':
    main()
