"""Preserve valid native base odometry as PoseStamped; never publish TF."""
from copy import deepcopy
import math
import signal

from geometry_msgs.msg import PoseStamped


def select_pose(msg, now_ns, last_stamp_ns=0, max_age_ns=1_000_000_000):
    """Return (pose, reason); no clock, ROS initialization or state mutation."""
    stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    if msg.header.frame_id != 'rtabmap_odom' or msg.child_frame_id != 'base_link':
        return None, 'frame'
    if not 0 <= msg.header.stamp.nanosec < 1_000_000_000:
        return None, 'stamp'
    if stamp <= last_stamp_ns or stamp <= 0:
        return None, 'stamp'
    if now_ns <= 0 or now_ns - stamp > max_age_ns or stamp - now_ns > 50_000_000:
        return None, 'age'
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    if not all(math.isfinite(v) for v in [p.x, p.y, p.z, q.x, q.y, q.z, q.w]):
        return None, 'nonfinite'
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if abs(norm - 1.0) > 0.001:
        return None, 'quaternion'
    result = PoseStamped()
    result.header = deepcopy(msg.header)
    result.pose = deepcopy(msg.pose.pose)
    return result, 'valid'


def finish_node(node, ros):
    # ros2 launch and a process-group stop can both deliver SIGINT. Teardown
    # must finish when the second signal arrives; it must not change actuation.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    node.destroy_node()
    if ros.ok():
        ros.shutdown()


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException

    class PoseAdapter(Node):
        def __init__(self):
            super().__init__('pose_adapter')
            self.last_stamp = 0
            self.publisher = self.create_publisher(PoseStamped, 'pose', qos_profile_sensor_data)
            self.subscription = self.create_subscription(
                Odometry, 'odom', self.receive, qos_profile_sensor_data)
            self.get_logger().info('Local base pose: rtabmap_odom; no TF broadcasting')

        def receive(self, msg):
            pose, reason = select_pose(msg, self.get_clock().now().nanoseconds, self.last_stamp)
            if pose is None:
                self.get_logger().warning('Dropped odometry: ' + reason, throttle_duration_sec=5.0)
                return
            self.last_stamp = pose.header.stamp.sec * 1_000_000_000 + pose.header.stamp.nanosec
            self.publisher.publish(pose)

    rclpy.init(args=args)
    node = PoseAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        finish_node(node, rclpy)
