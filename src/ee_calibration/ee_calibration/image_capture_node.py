#!/usr/bin/env python3
"""Jetson node: reacts to /calibration/pose_ready published by Desktop's
pose_sampler_node, grabs the color frame nearest to the settle timestamp,
saves it, and immediately records p_base + image path as one manifest entry.
This node never commands robot motion and never calls MoveIt2 -- it only
reacts to pose_ready and publishes capture_done.

Reuses hand_detector's camera topic conventions (color image + color
camera_info, qos_profile_sensor_data) without modifying hand_detector itself.
Depth back-projection style (for solve_calibration.py --cross-check-3d3d) is
mirrored from hand_detector_node.py's manual pinhole formula using
CameraInfo.k directly, not pyrealsense2.
"""
import json
import os
from collections import deque
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from ee_calibration_msgs.msg import PoseReady, CaptureDone


class ImageCaptureNode(Node):

    def __init__(self):
        super().__init__('image_capture_node')

        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('capture_depth', True)
        self.declare_parameter('image_sync_tolerance_sec', 0.2)
        self.declare_parameter('output_root', os.path.expanduser('~/calib_ws/calib_data'))
        self.declare_parameter('manifest_filename', 'manifest.json')
        self.declare_parameter('buffer_size', 60)
        self.declare_parameter('run_dir', '')  # empty -> auto timestamp

        p = self.get_parameter
        self._color_topic = p('color_topic').value
        self._camera_info_topic = p('camera_info_topic').value
        self._depth_topic = p('depth_topic').value
        self._capture_depth = bool(p('capture_depth').value)
        self._sync_tol = float(p('image_sync_tolerance_sec').value)
        # os.path.expanduser here, not just on the declared default: a
        # --params-file override supplies the literal string "~/..." and
        # ROS2 params don't shell-expand it, so without this the node
        # creates a real directory named "~" under its launch cwd instead
        # of the actual home dir (found 2026-08-27 after the first
        # successful capture landed at calib_ws/~/react_ws/calib_data/...).
        self._output_root = os.path.expanduser(p('output_root').value)
        self._manifest_filename = p('manifest_filename').value
        self._buffer_size = int(p('buffer_size').value)

        run_dir_param = p('run_dir').value
        run_name = run_dir_param if run_dir_param else datetime.now().strftime('%Y%m%d_%H%M%S')
        self._run_dir = os.path.join(self._output_root, run_name)
        self._images_dir = os.path.join(self._run_dir, 'images')
        self._depth_dir = os.path.join(self._run_dir, 'depth')
        os.makedirs(self._images_dir, exist_ok=True)
        if self._capture_depth:
            os.makedirs(self._depth_dir, exist_ok=True)
        self._manifest_path = os.path.join(self._run_dir, self._manifest_filename)

        self._bridge = CvBridge()
        self._color_buffer = deque(maxlen=self._buffer_size)  # (stamp_sec, cv_image)
        self._depth_buffer = deque(maxlen=self._buffer_size)
        self._camera_info_cached = None

        self._manifest = {
            'run_dir': self._run_dir,
            'camera_info': None,
            'entries': [],
        }
        if os.path.exists(self._manifest_path):
            # Resuming an existing run_dir (e.g. after a restart) -- keep
            # whatever is already on disk (including any external edits
            # like ee_click_tool.py's clicks) instead of blanking it.
            try:
                with open(self._manifest_path) as f:
                    on_disk = json.load(f)
                self._manifest['camera_info'] = on_disk.get('camera_info')
                self._manifest['entries'] = on_disk.get('entries', [])
                self._camera_info_cached = self._manifest['camera_info']
            except (OSError, json.JSONDecodeError):
                pass
        self._save_manifest()

        self.create_subscription(
            Image, self._color_topic, self._on_color, qos_profile_sensor_data)
        if self._capture_depth:
            self.create_subscription(
                Image, self._depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self._camera_info_topic, self._on_camera_info, qos_profile_sensor_data)

        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.history = HistoryPolicy.KEEP_LAST
        self._capture_done_pub = self.create_publisher(
            CaptureDone, '/calibration/capture_done', reliable_qos)
        self.create_subscription(
            PoseReady, '/calibration/pose_ready', self._on_pose_ready, reliable_qos)

        self.get_logger().info(f'image_capture_node writing manifest to {self._manifest_path}')

    # ---- buffering ---------------------------------------------------

    @staticmethod
    def _stamp_to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _on_color(self, msg: Image):
        cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._color_buffer.append((self._stamp_to_sec(msg.header.stamp), cv_img))

    def _on_depth(self, msg: Image):
        cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self._depth_buffer.append((self._stamp_to_sec(msg.header.stamp), cv_img))

    def _on_camera_info(self, msg: CameraInfo):
        if self._camera_info_cached is not None:
            return
        self._camera_info_cached = {
            'width': msg.width,
            'height': msg.height,
            'k': list(msg.k),
            'd': list(msg.d),
            'distortion_model': msg.distortion_model,
        }
        self._manifest['camera_info'] = self._camera_info_cached
        self._save_manifest()
        self.get_logger().info(
            f'Cached camera_info ({msg.width}x{msg.height}, '
            f'fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f})')

    def _nearest(self, buffer, target_sec):
        best = None
        best_dt = None
        for stamp_sec, img in buffer:
            dt = abs(stamp_sec - target_sec)
            if best_dt is None or dt < best_dt:
                best_dt = dt
                best = img
        if best is not None and best_dt is not None and best_dt <= self._sync_tol:
            return best, best_dt
        return None, best_dt

    # ---- manifest ------------------------------------------------------

    def _save_manifest(self):
        tmp_path = self._manifest_path + '.tmp'
        with open(tmp_path, 'w') as f:
            json.dump(self._manifest, f, indent=2)
        os.replace(tmp_path, self._manifest_path)

    # ---- pose_ready handling -------------------------------------------

    def _reload_entries_from_disk(self):
        """Merge in whatever's currently on disk before appending.

        _save_manifest() overwrites the whole file from this node's
        in-memory copy, which would silently wipe out edits an external
        tool (ee_click_tool.py) made to already-written entries -- found
        2026-08-27 when a 12-pose run erased 5 already-clicked entries.
        """
        if not os.path.exists(self._manifest_path):
            return
        try:
            with open(self._manifest_path) as f:
                on_disk = json.load(f)
        except (OSError, json.JSONDecodeError):
            return
        self._manifest['entries'] = on_disk.get('entries', self._manifest['entries'])

    def _on_pose_ready(self, msg: PoseReady):
        self._reload_entries_from_disk()
        target_sec = self._stamp_to_sec(msg.header.stamp)
        color_img, color_dt = self._nearest(self._color_buffer, target_sec)

        success = color_img is not None and self._camera_info_cached is not None
        entry = {
            'pose_index': msg.pose_index,
            'p_base': {'x': msg.p_base.x, 'y': msg.p_base.y, 'z': msg.p_base.z},
            'is_validation': msg.is_validation,
            'settle_stamp_sec': target_sec,
            'success': success,
            'image_path': None,
            'depth_path': None,
            'capture_dt_sec': color_dt,
            'pixel_uv': None,
            'click_status': 'pending',
        }

        if success:
            # Use the entry's position in the manifest, NOT msg.pose_index,
            # for the filename. pose_index restarts at 0 on every separate
            # pose_sampler_node invocation, so naming by it alone let
            # multiple batches silently overwrite each other's images on
            # disk while the manifest kept distinct (and now-mismatched)
            # p_base entries pointing at the same overwritten file --
            # found 2026-08-27 after multi-batch collection produced a
            # 106px RMS reprojection error (the operator was clicking the
            # same overwritten photo for several different real poses).
            entry_seq = len(self._manifest['entries'])
            image_name = f'entry_{entry_seq:04d}_pose_{msg.pose_index:03d}.png'
            image_path = os.path.join(self._images_dir, image_name)
            cv2.imwrite(image_path, color_img)
            entry['image_path'] = image_path

            if self._capture_depth:
                depth_img, depth_dt = self._nearest(self._depth_buffer, target_sec)
                if depth_img is not None:
                    depth_name = f'entry_{entry_seq:04d}_pose_{msg.pose_index:03d}_depth.png'
                    depth_path = os.path.join(self._depth_dir, depth_name)
                    cv2.imwrite(depth_path, depth_img)
                    entry['depth_path'] = depth_path

            self.get_logger().info(
                f'Pose {msg.pose_index}: captured (dt={color_dt:.3f}s) -> {image_path}')
        else:
            reason = 'no camera_info yet' if self._camera_info_cached is None else (
                f'no frame within tolerance (closest dt={color_dt})' if color_dt is not None
                else 'no frames buffered')
            self.get_logger().warn(f'Pose {msg.pose_index}: capture FAILED ({reason})')

        self._manifest['entries'].append(entry)
        self._save_manifest()

        done = CaptureDone()
        done.pose_index = msg.pose_index
        done.success = success
        self._capture_done_pub.publish(done)


def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
