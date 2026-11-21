"""Metric SE(3) pose comparison; no ROS node, optimizer, scale fit or SLAM code."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def transform(position, quaternion):
    p, q = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
        raise ValueError('nonfinite or malformed pose')
    if abs(np.linalg.norm(q) - 1) > 1e-3:
        raise ValueError('invalid quaternion norm')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(q).as_matrix()
    result[:3, 3] = p
    return result


def interpolate_reference(reference_times, reference_poses, query_times, max_gap=.10):
    times = np.asarray(reference_times, dtype=float)
    poses = np.asarray(reference_poses, dtype=float)
    query = np.asarray(query_times, dtype=float)
    if len(times) < 2 or not np.all(np.diff(times) > 0):
        raise ValueError('reference stamps must increase strictly')
    keep, output, brackets = [], [], []
    for i, stamp in enumerate(query):
        if stamp < times[0] or stamp > times[-1]:
            continue  # Never extrapolate.
        right = min(int(np.searchsorted(times, stamp, side='right')), len(times) - 1)
        left = max(0, right - 1)
        if times[right] - times[left] > max_gap:
            continue
        weight = (stamp - times[left]) / (times[right] - times[left])
        T = np.eye(4)
        T[:3, 3] = (1 - weight) * poses[left, :3, 3] + weight * poses[right, :3, 3]
        T[:3, :3] = Slerp(times[[left, right]], Rotation.from_matrix(poses[[left, right], :3, :3]))([stamp]).as_matrix()[0]
        keep.append(i)
        output.append(T)
        brackets.append(times[right] - times[left])
    return np.array(keep, dtype=int), np.array(output), np.array(brackets)


def stationary_alignment(estimated, reference):
    """Average relative orientation and translation over an initial stationary hold."""
    estimated, reference = np.asarray(estimated), np.asarray(reference)
    if len(estimated) < 20 or len(estimated) != len(reference):
        raise ValueError('insufficient stationary calibration pairs')
    R = Rotation.from_matrix(reference[:, :3, :3] @ estimated[:, :3, :3].transpose(0, 2, 1)).mean().as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.mean(reference[:, :3, 3] - estimated[:, :3, 3] @ R.T, axis=0)
    return T


def absolute_errors(estimated_aligned, reference):
    estimated_aligned, reference = np.asarray(estimated_aligned), np.asarray(reference)
    translation = np.linalg.norm(estimated_aligned[:, :3, 3] - reference[:, :3, 3], axis=1)
    rotation = Rotation.from_matrix(reference[:, :3, :3].transpose(0, 2, 1) @ estimated_aligned[:, :3, :3]).magnitude()
    return translation, rotation


def relative_errors(times, estimated, reference, interval=1., tolerance=.03):
    times, estimated, reference = np.asarray(times), np.asarray(estimated), np.asarray(reference)
    pairs, translation, rotation = [], [], []
    for i, stamp in enumerate(times):
        target = stamp + interval
        at = int(np.searchsorted(times, target))
        candidates = [j for j in [at - 1, at] if i < j < len(times)]
        if not candidates:
            continue
        j = min(candidates, key=lambda k: abs(times[k] - target))
        if abs(times[j] - target) > tolerance:
            continue
        delta_ref = np.linalg.inv(reference[i]) @ reference[j]
        delta_est = np.linalg.inv(estimated[i]) @ estimated[j]
        error = np.linalg.inv(delta_ref) @ delta_est
        pairs.append((i, j))
        translation.append(np.linalg.norm(error[:3, 3]))
        rotation.append(Rotation.from_matrix(error[:3, :3]).magnitude())
    return np.asarray(pairs, dtype=int), np.asarray(translation), np.asarray(rotation)


def statistics(values):
    values = np.asarray(values, dtype=float)
    if not len(values) or not np.isfinite(values).all():
        raise ValueError('no finite error values')
    return {'count': len(values), 'rmse': float(np.sqrt(np.mean(values ** 2))),
            'median': float(np.median(values)), 'p95': float(np.percentile(values, 95)),
            'max': float(values.max())}
