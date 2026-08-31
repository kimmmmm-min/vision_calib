#!/usr/bin/env python3
"""Republishes urcb2_driver's unnamed JointState as a standard, named
/joint_states so robot_state_publisher can build TF from it.

urcb2_driver (~/ros2_ws/src/urcb2_driver on Desktop) never calls
UrDriver::setJointNames(), so /UR10_right/joint_states always has an empty
`name` field even though `position` is a valid 6-element array. The array
order is a hardware/firmware fact, not something this code can verify -- it
must be confirmed once by physically jogging each joint and checking which
array index moves (see package README). Until that is done, do not trust any
FK/TF derived from this bridge.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

DEFAULT_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


class JointStateBridgeNode(Node):

    def __init__(self):
        super().__init__('joint_state_bridge_node')

        self.declare_parameter('source_topic', '/UR10_right/joint_states')
        self.declare_parameter('target_topic', '/joint_states')
        self.declare_parameter('joint_names', DEFAULT_JOINT_NAMES)

        self._source_topic = self.get_parameter('source_topic').value
        self._target_topic = self.get_parameter('target_topic').value
        self._joint_names = list(self.get_parameter('joint_names').value)

        if len(self._joint_names) != 6:
            raise ValueError(
                f'joint_names must have exactly 6 entries, got {len(self._joint_names)}')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        self._pub = self.create_publisher(JointState, self._target_topic, qos)
        self._sub = self.create_subscription(
            JointState, self._source_topic, self._on_source, qos)

        self._warned_size_mismatch = False
        self.get_logger().info(
            f'Bridging {self._source_topic} -> {self._target_topic} with joint_names='
            f'{self._joint_names} (VERIFY this order against real hardware before trusting FK)')

    def _on_source(self, msg: JointState):
        if len(msg.position) < len(self._joint_names):
            if not self._warned_size_mismatch:
                self.get_logger().warn(
                    f'Source joint_states has {len(msg.position)} positions, '
                    f'expected >= {len(self._joint_names)}; dropping until fixed')
                self._warned_size_mismatch = True
            return
        self._warned_size_mismatch = False

        out = JointState()
        out.header = msg.header
        if not out.header.stamp.sec and not out.header.stamp.nanosec:
            out.header.stamp = self.get_clock().now().to_msg()
        out.name = self._joint_names
        out.position = list(msg.position[:6])
        out.velocity = list(msg.velocity[:6]) if len(msg.velocity) >= 6 else []
        out.effort = list(msg.effort[:6]) if len(msg.effort) >= 6 else []
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
