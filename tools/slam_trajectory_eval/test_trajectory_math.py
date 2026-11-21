import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from trajectory_math import (transform, interpolate_reference, stationary_alignment,
                             absolute_errors, relative_errors)


class TrajectoryTest(unittest.TestCase):
    def test_known_rigid_frame_offset_removed_without_scale(self):
        offset = transform([4, -3, .4], Rotation.from_euler('xyz', [.2, -.1, .6]).as_quat())
        reference = np.repeat(np.eye(4)[None], 30, axis=0)
        estimated = np.linalg.inv(offset)[None] @ reference
        alignment = stationary_alignment(estimated, reference)
        self.assertTrue(np.allclose(alignment, offset, atol=1e-12))
        trans, rot = absolute_errors(alignment[None] @ estimated, reference)
        self.assertLess(trans.max(), 1e-12)
        self.assertLess(rot.max(), 1e-12)

    def test_slerp_translation_and_no_extrapolation_or_large_gaps(self):
        poses = [transform([0, 0, 0], [0, 0, 0, 1]),
                 transform([1, 0, 0], Rotation.from_euler('z', np.pi).as_quat())]
        keep, out, _ = interpolate_reference([0, 1], poses, [-.1, .5, 1.1], max_gap=1)
        self.assertEqual(keep.tolist(), [1])
        self.assertAlmostEqual(out[0, 0, 3], .5)
        self.assertTrue(np.allclose(out[0, :3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-12))
        self.assertEqual(len(interpolate_reference([0, 1], poses, [.5], max_gap=.1)[0]), 0)

    def test_one_second_rpe_detects_known_linear_drift(self):
        times = np.arange(31) / 10
        ref = np.repeat(np.eye(4)[None], len(times), axis=0)
        ref[:, 0, 3] = times * .1
        est = ref.copy(); est[:, 0, 3] += times * .01
        pairs, distance, angle = relative_errors(times, est, ref)
        self.assertEqual(len(pairs), 21)
        self.assertTrue(np.allclose(distance, .01))
        self.assertTrue(np.allclose(angle, 0))

    def test_stationary_alignment_does_not_fit_away_scale_error(self):
        hold = np.repeat(np.eye(4)[None], 30, axis=0)
        align = stationary_alignment(hold, hold)
        ref = np.repeat(np.eye(4)[None], 21, axis=0); ref[:, 0, 3] = np.arange(21) / 10
        est = ref.copy(); est[:, 0, 3] *= 2
        distance, _ = absolute_errors(align[None] @ est, ref)
        self.assertAlmostEqual(distance[-1], 2)


if __name__ == '__main__':
    unittest.main()
