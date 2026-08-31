#!/usr/bin/env python3
"""Publish the fixed, physically identifiable calibration point frame.

The point is configured as a fixed offset from ``tool0``.  Keeping this
separate from the MoveIt end-effector link lets MoveIt continue planning for
``tool0`` while calibration uses the exact physical point selected in the
camera image.
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class CalibrationPointPublisher(Node):

    def __init__(self):
        super().__init__('calibration_point_publisher')
        self.declare_parameter('parent_frame', 'tool0')
        self.declare_parameter('child_frame', 'calibration_point')
        self.declare_parameter('translation_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('rotation_rpy', [0.0, 0.0, 0.0])

        parent_frame = str(self.get_parameter('parent_frame').value)
        child_frame = str(self.get_parameter('child_frame').value)
        translation = list(self.get_parameter('translation_xyz').value)
        rotation = list(self.get_parameter('rotation_rpy').value)

        if len(translation) != 3:
            raise ValueError('translation_xyz must contain exactly 3 values')
        if len(rotation) != 3:
            raise ValueError('rotation_rpy must contain exactly 3 values')
        if not parent_frame or not child_frame:
            raise ValueError('parent_frame and child_frame must not be empty')
        if parent_frame == child_frame:
            raise ValueError('parent_frame and child_frame must be different')

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        qx, qy, qz, qw = _quaternion_from_rpy(*map(float, rotation))
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self._broadcaster = StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(transform)
        self.get_logger().info(
            f'Calibration point TF: {parent_frame} -> {child_frame}, '
            f'xyz={[float(v) for v in translation]} m, '
            f'rpy={[float(v) for v in rotation]} rad')


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationPointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
