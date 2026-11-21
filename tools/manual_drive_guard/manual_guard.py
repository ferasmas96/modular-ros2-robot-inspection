#!/usr/bin/env python3
"""Clamp only operator input; default-disarmed, simulation-stamped manual relay."""
import json
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String
from std_srvs.srv import SetBool

from control_limits import ManualGate, ZERO, bounded

NS = '/a300_00000'
RAW = NS + '/manual/raw_cmd_vel'
PREVIEW = NS + '/manual/limited_preview'
OUTPUT = NS + '/cmd_vel'
STATUS = NS + '/manual/guard_status'
ARM = NS + '/manual/arm'
OTHER_INPUTS = [NS + '/' + name for name in [
    'joy_teleop/cmd_vel', 'rc_teleop/cmd_vel', 'twist_marker_server/cmd_vel']]


class ManualGuard(Node):
    def __init__(self):
        super().__init__('slam004c_manual_guard', namespace=NS,
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.gate = ManualGate()
        self.preview_pub = self.create_publisher(TwistStamped, PREVIEW, 10)
        self.output_pub = self.create_publisher(TwistStamped, OUTPUT, 10)
        self.status_pub = self.create_publisher(String, STATUS, 10)
        self.raw_sub = self.create_subscription(TwistStamped, RAW, self.input, qos_profile_sensor_data)
        self.clock_sub = self.create_subscription(ClockMessage, '/clock', self.clock_input, qos_profile_sensor_data)
        self.service = self.create_service(SetBool, ARM, self.set_armed)
        self.last_graph = 0.0
        self.graph_ok = False
        self.graph_reason = 'discovery not checked'
        self.last_status = 0.0
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.05, self.tick, clock=self.wall_clock)
        self.get_logger().info('DISARMED manual relay: hard limits 0.10m/s and 0.10rad/s; no autonomous commands')

    def clock_input(self, msg):
        self.gate.clock(msg.clock.sec * 10**9 + msg.clock.nanosec, time.monotonic())

    def input(self, msg):
        t = msg.twist
        self.gate.receive((t.linear.x, t.linear.y, t.linear.z,
                           t.angular.x, t.angular.y, t.angular.z), time.monotonic())

    def check_graph(self):
        own = NS + '/slam004c_manual_guard'
        names = lambda topic: [e.node_namespace.rstrip('/') + '/' + e.node_name
                               for e in self.get_publishers_info_by_topic(topic)]
        if names(OUTPUT) != [own]:
            return False, 'manual mux input is not exclusive to this guard'
        if names(NS + '/platform/cmd_vel') != [NS + '/twist_mux']:
            return False, 'unexpected controller-input publisher'
        if names(RAW) != [NS + '/slam004c_manual_input_bridge']:
            return False, 'raw manual input bridge is missing/nonexclusive'
        for topic in OTHER_INPUTS:
            if names(topic):
                return False, 'another mux input publisher exists: ' + topic
        return True, 'exclusive manual path verified'

    def set_armed(self, request, response):
        if not request.data:
            self.gate.disarm()
            response.success, response.message = True, 'disarmed; only zero robot commands'
        else:
            self.graph_ok, self.graph_reason = self.check_graph()
            response.success = self.graph_ok and self.gate.arm(time.monotonic())
            response.message = ('armed; waiting for fresh operator input' if response.success
                                else 'refused: ' + (self.gate.fault or self.graph_reason + '; release all keys/check clock'))
        self.get_logger().info(response.message)
        return response

    def message(self, values):
        assert bounded(values) == values
        m = TwistStamped()
        stamp = self.gate.clock_ns or 0
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, 10**9)
        m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.angular.z = values[0], values[5]
        return m

    def tick(self):
        now = time.monotonic()
        if now - self.last_graph >= 0.2:
            self.last_graph = now
            self.graph_ok, self.graph_reason = self.check_graph()
            if self.gate.armed and not self.graph_ok:
                self.gate.fail(self.graph_reason)
        preview, output = self.gate.sample(now)
        if self.gate.clock_ns is not None:
            self.preview_pub.publish(self.message(preview))
            self.output_pub.publish(self.message(output))
        if now - self.last_status >= 0.5:
            self.last_status = now
            m = String()
            m.data = json.dumps({'clock_ns': self.gate.clock_ns, 'armed': self.gate.armed,
                                 'fault': self.gate.fault, 'graph_ok': self.graph_ok,
                                 'graph_reason': self.graph_reason,
                                 'preview': list(preview), 'output': list(output)})
            self.status_pub.publish(m)


def main():
    rclpy.init()
    node = ManualGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.gate.disarm()
        if rclpy.ok():
            node.output_pub.publish(node.message(ZERO))
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
