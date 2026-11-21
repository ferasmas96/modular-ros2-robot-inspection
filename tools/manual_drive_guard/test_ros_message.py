"""Serialization checks only: no ROS init, node or publisher is constructed."""
import unittest
from types import SimpleNamespace
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.topic_endpoint_info import TopicEndpointInfo
from geometry_msgs.msg import TwistStamped
from manual_guard import ManualGuard, NS, OUTPUT, RAW, OTHER_INPUTS
from control_limits import ManualGate, bounded


class WireMessageTest(unittest.TestCase):
    def test_actual_message_builder_uses_sensor_clock_and_six_bounded_components(self):
        g = ManualGate(); g.clock(13234789012, 1)
        fake = SimpleNamespace(gate=g)
        m = ManualGuard.message(fake, bounded((1, 2, 3, 4, 5, .5)))
        m = deserialize_message(serialize_message(m), TwistStamped)
        self.assertEqual(m.header.stamp.sec, 13)
        self.assertEqual(m.header.stamp.nanosec, 234789012)
        self.assertEqual(m.header.frame_id, 'base_link')
        self.assertEqual((m.twist.linear.x, m.twist.linear.y, m.twist.linear.z,
                          m.twist.angular.x, m.twist.angular.y, m.twist.angular.z),
                         (.1, 0, 0, 0, 0, .1))

    def test_message_builder_refuses_unbounded_tuple(self):
        fake = SimpleNamespace(gate=ManualGate())
        with self.assertRaises(AssertionError):
            ManualGuard.message(fake, (.10001, 0, 0, 0, 0, 0))


class EndpointGraphTest(unittest.TestCase):
    def setUp(self):
        def endpoint(name):
            return TopicEndpointInfo(node_namespace=NS, node_name=name)
        self.endpoint = endpoint
        self.graph = {
            OUTPUT: [endpoint('slam004c_manual_guard')],
            RAW: [endpoint('slam004c_manual_input_bridge')],
            NS + '/platform/cmd_vel': [endpoint('twist_mux')],
        }
        self.node = SimpleNamespace(get_publishers_info_by_topic=lambda t: self.graph.get(t, []))

    def test_installed_endpoint_properties_and_exclusive_path(self):
        self.assertTrue(ManualGuard.check_graph(self.node)[0])

    def test_bypass_publisher_refuses_arm(self):
        self.graph[OUTPUT].append(self.endpoint('cmd_vel_bridge'))
        self.assertFalse(ManualGuard.check_graph(self.node)[0])

    def test_missing_raw_bridge_refuses_arm(self):
        self.graph[RAW] = []
        self.assertFalse(ManualGuard.check_graph(self.node)[0])

    def test_higher_priority_input_refuses_arm(self):
        self.graph[OTHER_INPUTS[0]] = [self.endpoint('teleop_twist_joy_node')]
        self.assertFalse(ManualGuard.check_graph(self.node)[0])


if __name__ == '__main__':
    unittest.main()
