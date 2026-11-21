import math
import unittest
import signal
from nav_msgs.msg import Odometry
from rtabmap_husky_integration.pose_adapter import select_pose, finish_node


class PoseSelectionTests(unittest.TestCase):
    def setUp(self):
        self.msg = Odometry()
        self.msg.header.frame_id = 'rtabmap_odom'
        self.msg.child_frame_id = 'base_link'
        self.msg.header.stamp.sec = 12
        self.msg.header.stamp.nanosec = 123456789
        self.stamp = 12_123456789
        self.msg.pose.pose.position.x = 1.25
        self.msg.pose.pose.orientation.w = 1.0

    def selected(self):
        return select_pose(self.msg, self.stamp + 20_000_000)

    def test_preserves_exact_sensor_stamp_frame_and_pose(self):
        pose, _ = self.selected()
        self.assertEqual(pose.header, self.msg.header)
        self.assertEqual(pose.pose, self.msg.pose.pose)
        self.msg.pose.pose.position.x = 99.0
        self.assertEqual(pose.pose.position.x, 1.25)

    def test_rejects_other_reference_frame(self):
        self.msg.header.frame_id = 'odom'
        self.assertIsNone(self.selected()[0])

    def test_rejects_sensor_body(self):
        self.msg.child_frame_id = 'camera_0_color_optical_frame'
        self.assertIsNone(self.selected()[0])

    def test_rejects_nan_and_infinity(self):
        for value in [math.nan, math.inf, -math.inf]:
            self.msg.pose.pose.position.y = value
            self.assertIsNone(self.selected()[0])

    def test_rejects_invalid_orientation(self):
        for value in [0.0, 1.1, math.nan, math.inf]:
            self.msg.pose.pose.orientation.w = value
            self.assertIsNone(self.selected()[0])

    def test_accepts_negative_unit_quaternion_without_changing_it(self):
        self.msg.pose.pose.orientation.w = -1.0
        self.assertEqual(self.selected()[0].pose.orientation.w, -1.0)

    def test_rejects_duplicate_and_decreasing_stamps(self):
        for previous in [self.stamp, self.stamp + 1]:
            self.assertIsNone(select_pose(self.msg, self.stamp, previous)[0])

    def test_rejects_zero_stamp(self):
        self.msg.header.stamp.sec = self.msg.header.stamp.nanosec = 0
        self.assertIsNone(self.selected()[0])

    def test_rejects_uninitialized_clock(self):
        self.assertIsNone(select_pose(self.msg, 0)[0])

    def test_rejects_old_and_future_estimates(self):
        for now in [self.stamp + 1_000_000_001, self.stamp - 50_000_001]:
            self.assertIsNone(select_pose(self.msg, now)[0])

    def test_retains_input_without_using_or_fabricating_covariance(self):
        self.msg.pose.covariance[0] = 0.1
        self.assertEqual(self.selected()[0].pose, self.msg.pose.pose)
        self.assertEqual(self.msg.pose.covariance[0], 0.1)

    def test_second_sigint_cannot_interrupt_teardown(self):
        calls = []

        class FakeNode:
            def destroy_node(self):
                signal.raise_signal(signal.SIGINT)
                calls.append('destroyed')

        class FakeROS:
            def ok(self):
                return True

            def shutdown(self):
                calls.append('shutdown')

        previous = signal.getsignal(signal.SIGINT)
        try:
            finish_node(FakeNode(), FakeROS())
            self.assertEqual(calls, ['destroyed', 'shutdown'])
        finally:
            signal.signal(signal.SIGINT, previous)


if __name__ == '__main__':
    unittest.main()
