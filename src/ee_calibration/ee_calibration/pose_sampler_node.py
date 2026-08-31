#!/usr/bin/env python3
"""Desktop node: generates EE calibration poses biased toward the
forearm/wrist working area, drives the robot there via MoveIt2's raw
MoveGroup action (neither moveit_commander nor moveit_py is installed on this
workspace -- see plan), reads the physical calibration point's p_base from a
local tf2 lookup once settled,
and publishes it to the Jetson over /calibration/pose_ready. Writes no result
files -- the Jetson's manifest is the sole source of truth.

Run modes:
  --dry-run    print the structured pose list, do not contact MoveIt
  --plan-only  ask MoveIt to plan and run all safety checks, but do not execute
  --step       require operator confirmation before each accepted plan executes
  --validate   use a smaller, distinct structured pose set and tag the samples
               is_validation=True
"""
import argparse
import math
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, PositionIKRequest, RobotState, WorkspaceParameters,
)
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from ee_calibration_msgs.msg import PoseReady, CaptureDone


def _quat_from_rpy(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def _rpy_from_quat(q):
    """Convert a geometry_msgs Quaternion to roll/pitch/yaw."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _camera_aligned_axes(camera_xyz, look_at_xyz):
    """Return camera-right, camera-up, and forward axes in the base frame."""
    forward = np.asarray(look_at_xyz, dtype=float) - np.asarray(camera_xyz, dtype=float)
    forward_norm = float(np.linalg.norm(forward))
    if forward_norm < 1e-6:
        raise ValueError('camera_approx_xyz and camera_approx_look_at_xyz must differ')
    forward /= forward_norm

    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, world_up)
    if np.linalg.norm(right) < 1e-6:
        right = np.cross(forward, np.array([0.0, 1.0, 0.0]))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    up /= np.linalg.norm(up)
    return right, up, forward


def _bounded_shortest_path_order(positions, start_xyz, max_step):
    """Find the shortest visit order using only edges no longer than max_step.

    The structured set contains at most 12 points, so an exact dynamic-programming
    Hamiltonian-path search is small and avoids the long final jump that a greedy
    nearest-neighbour walk can leave behind.
    """
    points = [np.asarray(p, dtype=float) for p in positions]
    count = len(points)
    if count == 0:
        return []
    start = np.asarray(start_xyz, dtype=float)
    distances = np.array([
        [float(np.linalg.norm(points[i] - points[j])) for j in range(count)]
        for i in range(count)
    ])
    start_distances = [float(np.linalg.norm(point - start)) for point in points]

    # (visited_mask, last_index) -> (total_distance, previous_index)
    best = {}
    for index, distance in enumerate(start_distances):
        if distance <= max_step + 1e-9:
            best[(1 << index, index)] = (distance, None)

    for mask in range(1, 1 << count):
        for last in range(count):
            state = best.get((mask, last))
            if state is None:
                continue
            total, _ = state
            for nxt in range(count):
                if mask & (1 << nxt) or distances[last, nxt] > max_step + 1e-9:
                    continue
                next_mask = mask | (1 << nxt)
                next_total = total + distances[last, nxt]
                previous_best = best.get((next_mask, nxt))
                if previous_best is None or next_total < previous_best[0]:
                    best[(next_mask, nxt)] = (next_total, last)

    full_mask = (1 << count) - 1
    endings = [
        (state[0], last) for (mask, last), state in best.items()
        if mask == full_mask
    ]
    if not endings:
        raise ValueError(
            f'no visit order connects all structured poses with max_step={max_step:.3f}m')

    _, last = min(endings)
    reverse_indices = []
    mask = full_mask
    while last is not None:
        reverse_indices.append(last)
        _, previous = best[(mask, last)]
        mask &= ~(1 << last)
        last = previous
    return [points[index] for index in reversed(reverse_indices)]


TRAINING_OFFSETS = [
    # Four points on the camera-near layer.
    (-1.0, -1.0, -1.0),
    (1.0, -1.0, -1.0),
    (1.0, 1.0, -1.0),
    (-1.0, 1.0, -1.0),
    # Four axis points on the centre-depth layer.
    (0.0, -1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (-1.0, 0.0, 0.0),
    # Four points on the camera-far layer.
    (-1.0, -1.0, 1.0),
    (1.0, -1.0, 1.0),
    (1.0, 1.0, 1.0),
    (-1.0, 1.0, 1.0),
]

VALIDATION_OFFSETS = [
    (-0.55, -0.45, 0.55),
    (0.55, -0.45, -0.55),
    (0.55, 0.45, 0.55),
    (-0.55, 0.45, -0.55),
]


class PoseSamplerNode(Node):

    def __init__(self, dry_run: bool, plan_only: bool, step: bool, validate: bool):
        super().__init__('pose_sampler_node')
        self._dry_run = dry_run
        self._plan_only = plan_only
        self._step = step
        self._validate = validate

        self.declare_parameter('num_poses', 4 if validate else 12)
        self.declare_parameter('validation_num_poses', 4)
        self.declare_parameter('base_frame', 'base')
        self.declare_parameter('ee_frame', 'tool0')
        self.declare_parameter('calibration_frame', 'calibration_point')
        self.declare_parameter('planning_group', 'ur_manipulator')
        self.declare_parameter('center_mode', 'current')
        self.declare_parameter('configured_center_xyz', [0.4, 0.0, 0.3])
        self.declare_parameter('orientation_mode', 'current')
        self.declare_parameter('nominal_rpy', [math.pi, 0.0, 0.0])  # tool0 pointing down
        self.declare_parameter('orientation_variation_rad', 0.0)
        self.declare_parameter('sampling_lateral_extent_m', 0.10)
        self.declare_parameter('sampling_vertical_extent_m', 0.08)
        self.declare_parameter('sampling_depth_extent_m', 0.06)
        self.declare_parameter('max_cartesian_step_m', 0.15)
        self.declare_parameter('settle_time_sec', 0.25)
        self.declare_parameter('motion_timeout_sec', 30.0)
        self.declare_parameter('capture_timeout_sec', 10.0)
        # Rough approximate camera pose guess in base frame, for the FOV
        # pre-filter only (chicken-and-egg: we don't have T_base_camera yet).
        # Physical setup: camera ~1.5-2m from the sampled workspace, looking
        # down at an angle.
        self.declare_parameter('camera_approx_xyz', [0.4, -1.5, 1.2])
        self.declare_parameter('camera_approx_look_at_xyz', [0.4, 0.0, 0.3])
        self.declare_parameter('camera_depth_min_m', 0.5)
        self.declare_parameter('camera_depth_max_m', 3.0)
        self.declare_parameter('camera_half_fov_deg', 35.0)
        self.declare_parameter('visibility_margin_deg', 7.0)
        self.declare_parameter(
            'visibility_links', ['forearm_link', 'wrist_1_link', 'tool0'])
        self.declare_parameter('fk_service_name', '/compute_fk')
        self.declare_parameter('ik_service_name', '/compute_ik')
        # Per-joint band (rad) MoveGroup is allowed to plan around the
        # IK solution seeded from the current joint state. Kept tight --
        # this is a landing-precision tolerance, not a pose tolerance.
        self.declare_parameter('joint_goal_tolerance_rad', 0.01)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('joint_names', [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'])
        self.declare_parameter(
            'max_joint_delta_rad', [0.30, 0.30, 0.35, 0.35, 0.30, 0.20])
        self.declare_parameter('settle_velocity_threshold_rad_s', 0.01)
        self.declare_parameter('settle_stable_time_sec', 0.5)
        self.declare_parameter('settle_timeout_sec', 4.0)
        # Kept low by default -- first real-hardware runs showed the robot
        # moving uncomfortably fast at 0.3 (30%). Raise only after watching a
        # few slow, controlled poses complete safely.
        self.declare_parameter('max_velocity_scaling_factor', 0.1)
        self.declare_parameter('max_acceleration_scaling_factor', 0.05)

        p = self.get_parameter
        self._num_poses = int(
            p('validation_num_poses').value if validate else p('num_poses').value)
        self._base_frame = p('base_frame').value
        self._ee_frame = p('ee_frame').value
        self._calibration_frame = p('calibration_frame').value
        self._planning_group = p('planning_group').value
        self._center_mode = str(p('center_mode').value)
        self._configured_center = np.array(p('configured_center_xyz').value, dtype=float)
        self._orientation_mode = str(p('orientation_mode').value)
        self._nominal_rpy = list(p('nominal_rpy').value)
        self._orient_var = float(p('orientation_variation_rad').value)
        self._lateral_extent = float(p('sampling_lateral_extent_m').value)
        self._vertical_extent = float(p('sampling_vertical_extent_m').value)
        self._sampling_depth_extent = float(p('sampling_depth_extent_m').value)
        self._max_cartesian_step = float(p('max_cartesian_step_m').value)
        self._settle_time_sec = float(p('settle_time_sec').value)
        self._motion_timeout_sec = float(p('motion_timeout_sec').value)
        self._capture_timeout_sec = float(p('capture_timeout_sec').value)
        self._cam_pos = np.array(p('camera_approx_xyz').value, dtype=float)
        self._cam_look_at = np.array(p('camera_approx_look_at_xyz').value, dtype=float)
        self._depth_min = float(p('camera_depth_min_m').value)
        self._depth_max = float(p('camera_depth_max_m').value)
        self._half_fov_rad = math.radians(float(p('camera_half_fov_deg').value))
        self._visibility_margin_rad = math.radians(float(p('visibility_margin_deg').value))
        self._visibility_links = list(p('visibility_links').value)
        self._fk_service_name = str(p('fk_service_name').value)
        self._ik_service_name = str(p('ik_service_name').value)
        self._joint_goal_tolerance = float(p('joint_goal_tolerance_rad').value)
        self._joint_states_topic = str(p('joint_states_topic').value)
        self._joint_names = list(p('joint_names').value)
        self._max_joint_delta = np.array(p('max_joint_delta_rad').value, dtype=float)
        self._settle_velocity_threshold = float(
            p('settle_velocity_threshold_rad_s').value)
        self._settle_stable_time = float(p('settle_stable_time_sec').value)
        self._settle_timeout = float(p('settle_timeout_sec').value)
        self._vel_scale = float(p('max_velocity_scaling_factor').value)
        self._accel_scale = float(p('max_acceleration_scaling_factor').value)

        if self._center_mode not in ('current', 'configured'):
            raise ValueError('center_mode must be "current" or "configured"')
        if self._orientation_mode not in ('current', 'configured'):
            raise ValueError('orientation_mode must be "current" or "configured"')
        if self._num_poses <= 0:
            raise ValueError('num_poses must be positive')
        if len(self._configured_center) != 3 or len(self._nominal_rpy) != 3:
            raise ValueError('configured_center_xyz and nominal_rpy must each have 3 values')
        if len(self._joint_names) != 6 or len(self._max_joint_delta) != 6:
            raise ValueError('joint_names and max_joint_delta_rad must each have 6 values')
        if min(self._lateral_extent, self._vertical_extent, self._sampling_depth_extent) <= 0.0:
            raise ValueError('all structured sampling extents must be positive')
        if self._visibility_margin_rad >= self._half_fov_rad:
            raise ValueError('visibility_margin_deg must be smaller than camera_half_fov_deg')

        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        reliable_qos.history = HistoryPolicy.KEEP_LAST

        self._pose_ready_pub = self.create_publisher(
            PoseReady, '/calibration/pose_ready', reliable_qos)
        self._capture_done_event = None
        self._capture_done_success = False
        self._capture_done_sub = self.create_subscription(
            CaptureDone, '/calibration/capture_done', self._on_capture_done, reliable_qos)

        self._joint_state_lock = threading.Lock()
        self._latest_joint_positions = None
        self._latest_joint_velocities = None
        self._latest_joint_state_time = None
        self._sampling_center = None
        self._joint_state_sub = self.create_subscription(
            JointState, self._joint_states_topic, self._on_joint_state, reliable_qos)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._move_client = ActionClient(self, MoveGroup, '/move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self._fk_client = self.create_client(GetPositionFK, self._fk_service_name)
        self._ik_client = self.create_client(GetPositionIK, self._ik_service_name)

    # ---- pose generation -------------------------------------------------

    def _camera_forward_axis(self):
        return _camera_aligned_axes(self._cam_pos, self._cam_look_at)[2]

    def _in_camera_fov(self, p_base_xyz: np.ndarray, margin_rad: float = 0.0) -> bool:
        rel = p_base_xyz - self._cam_pos
        depth = float(np.dot(rel, self._camera_forward_axis()))
        if not (self._depth_min <= depth <= self._depth_max):
            return False
        dist = np.linalg.norm(rel)
        if dist < 1e-6:
            return False
        angle = math.acos(max(-1.0, min(1.0, depth / dist)))
        return angle <= (self._half_fov_rad - margin_rad)

    def _lookup_current_ee_pose(self):
        deadline = time.monotonic() + 3.0
        last_error = None
        while time.monotonic() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._base_frame, self._ee_frame, rclpy.time.Time())
                t = transform.transform.translation
                xyz = np.array([t.x, t.y, t.z], dtype=float)
                rpy = _rpy_from_quat(transform.transform.rotation)
                return xyz, rpy
            except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                last_error = ex
                rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().error(
            f'Could not read current {self._base_frame}->{self._ee_frame} TF: {last_error}')
        return None

    def _select_structured_offsets(self):
        available = VALIDATION_OFFSETS if self._validate else TRAINING_OFFSETS
        if self._num_poses > len(available):
            raise ValueError(
                f'num_poses={self._num_poses} exceeds the structured '
                f'{"validation" if self._validate else "training"} set size '
                f'of {len(available)}')
        if self._num_poses == len(available):
            return available
        indices = np.linspace(0, len(available) - 1, self._num_poses)
        return [available[int(round(i))] for i in indices]

    def generate_poses(self):
        current_pose = None
        if self._center_mode == 'current' or self._orientation_mode == 'current':
            current_pose = self._lookup_current_ee_pose()
            if current_pose is None:
                raise RuntimeError(
                    'Current-pose sampling requires live base->tool0 TF; start '
                    'desktop_bridge and urcb2_driver before generating poses')

        center = current_pose[0] if self._center_mode == 'current' else self._configured_center
        nominal_rpy = current_pose[1] if self._orientation_mode == 'current' else self._nominal_rpy
        right, up, forward = _camera_aligned_axes(self._cam_pos, self._cam_look_at)

        candidates = []
        for lateral_n, vertical_n, depth_n in self._select_structured_offsets():
            xyz = (
                center
                + lateral_n * self._lateral_extent * right
                + vertical_n * self._vertical_extent * up
                + depth_n * self._sampling_depth_extent * forward
            )
            if not self._in_camera_fov(xyz):
                self.get_logger().warn(
                    f'Rejecting structured pose outside approximate camera FOV: '
                    f'xyz={xyz.tolist()}')
                continue

            # Usually orient_var is zero.  If deliberately enabled later, use
            # structured, bounded variations rather than random wrist motion.
            rpy = (
                nominal_rpy[0] + depth_n * self._orient_var,
                nominal_rpy[1] + vertical_n * self._orient_var,
                nominal_rpy[2] + lateral_n * self._orient_var,
            )
            candidates.append((xyz, rpy))

        if len(candidates) != self._num_poses:
            raise RuntimeError(
                f'Only {len(candidates)}/{self._num_poses} structured poses are '
                'inside the approximate camera FOV; adjust the camera estimate '
                'or reduce the sampling extents before moving the robot')

        ordered_xyz = _bounded_shortest_path_order(
            [xyz for xyz, _ in candidates], center, self._max_cartesian_step)
        pose_by_xyz = {tuple(xyz.tolist()): rpy for xyz, rpy in candidates}
        poses = [(xyz, pose_by_xyz[tuple(xyz.tolist())]) for xyz in ordered_xyz]

        previous = center
        for index, (xyz, _) in enumerate(poses):
            step = float(np.linalg.norm(xyz - previous))
            if step > self._max_cartesian_step + 1e-9:
                raise RuntimeError(
                    f'Structured pose {index} requires a {step:.3f}m Cartesian '
                    f'step, exceeding max_cartesian_step_m={self._max_cartesian_step:.3f}; '
                    'reduce extents or add intermediate structured poses')
            previous = xyz

        self.get_logger().info(
            f'Structured pose centre={center.tolist()}, current_orientation_rpy='
            f'{list(map(float, nominal_rpy))}, extents(camera L/V/D)='
            f'[{self._lateral_extent:.3f}, {self._vertical_extent:.3f}, '
            f'{self._sampling_depth_extent:.3f}]m')
        self._sampling_center = center.copy()
        return poses

    # ---- MoveGroup (raw action, no moveit_commander/moveit_py installed) --

    def _on_joint_state(self, msg: JointState):
        if not msg.name:
            return
        try:
            indices = [msg.name.index(name) for name in self._joint_names]
        except ValueError:
            return
        if any(index >= len(msg.position) for index in indices):
            return
        now = time.monotonic()
        positions = np.array([msg.position[index] for index in indices], dtype=float)
        velocities = None
        if msg.velocity and all(index < len(msg.velocity) for index in indices):
            velocities = np.array([msg.velocity[index] for index in indices], dtype=float)
        with self._joint_state_lock:
            if (
                velocities is None
                and self._latest_joint_positions is not None
                and self._latest_joint_state_time is not None
                and now > self._latest_joint_state_time
            ):
                velocities = (
                    positions - self._latest_joint_positions
                ) / (now - self._latest_joint_state_time)
            self._latest_joint_positions = positions
            self._latest_joint_velocities = velocities
            self._latest_joint_state_time = now

    def _get_joint_state(self):
        with self._joint_state_lock:
            positions = (
                None if self._latest_joint_positions is None
                else self._latest_joint_positions.copy())
            velocities = (
                None if self._latest_joint_velocities is None
                else self._latest_joint_velocities.copy())
            stamp = self._latest_joint_state_time
        return positions, velocities, stamp

    def _wait_for_joint_state(self, timeout_sec=3.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            positions, _, stamp = self._get_joint_state()
            if positions is not None and stamp is not None and time.monotonic() - stamp < 0.5:
                return positions
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def _compute_ik_near(self, xyz, rpy, seed_positions):
        """Solve IK for (xyz, rpy) seeded from seed_positions, so the result
        stays in the same branch as the current configuration instead of an
        arbitrary valid solution (wrist flip, shoulder/elbow flip, etc.) --
        see DESKTOP_CALIB_SETUP_LOG.md 2026-08-27 for the failure this fixes.
        """
        if not self._ik_client.wait_for_service(timeout_sec=3.0):
            return None, f'IK service {self._ik_service_name} is unavailable'

        pose = PoseStamped()
        pose.header.frame_id = self._base_frame
        pose.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
        pose.pose.orientation = _quat_from_rpy(*rpy)

        request = GetPositionIK.Request()
        request.ik_request = PositionIKRequest()
        request.ik_request.group_name = self._planning_group
        request.ik_request.robot_state = RobotState()
        request.ik_request.robot_state.joint_state.name = self._joint_names
        request.ik_request.robot_state.joint_state.position = seed_positions.tolist()
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout.sec = 1
        request.ik_request.pose_stamped = pose

        future = self._ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None:
            return None, 'IK request timed out'
        if response.error_code.val != 1:  # moveit_msgs/MoveItErrorCodes.SUCCESS
            return None, f'IK failed with error_code={response.error_code.val}'

        solution = dict(zip(
            response.solution.joint_state.name, response.solution.joint_state.position))
        try:
            positions = np.array([solution[name] for name in self._joint_names], dtype=float)
        except KeyError as ex:
            return None, f'IK solution missing joint {ex}'
        return positions, None

    def _build_joint_goal(self, joint_positions, plan_only=True):
        constraints = Constraints()
        constraints.joint_constraints = [
            JointConstraint(
                joint_name=name, position=float(target),
                tolerance_above=self._joint_goal_tolerance,
                tolerance_below=self._joint_goal_tolerance, weight=1.0)
            for name, target in zip(self._joint_names, joint_positions)
        ]

        goal = MoveGroup.Goal()
        goal.request.group_name = self._planning_group
        goal.request.goal_constraints = [constraints]
        goal.request.allowed_planning_time = max(1.0, self._motion_timeout_sec * 0.6)
        goal.request.num_planning_attempts = 3
        goal.request.max_velocity_scaling_factor = self._vel_scale
        goal.request.max_acceleration_scaling_factor = self._accel_scale
        goal.request.workspace_parameters = WorkspaceParameters()
        goal.request.workspace_parameters.header.frame_id = self._base_frame
        goal.request.workspace_parameters.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        goal.request.workspace_parameters.max_corner = Vector3(x=1.0, y=1.0, z=1.5)
        goal.planning_options.plan_only = plan_only
        return goal

    def _plan_pose(self, xyz, rpy, seed_positions):
        if not self._move_client.wait_for_server(timeout_sec=self._motion_timeout_sec):
            self.get_logger().error('/move_action server not available')
            return None

        joint_target, ik_error = self._compute_ik_near(xyz, rpy, seed_positions)
        if joint_target is None:
            self.get_logger().warn(f'IK seeded from current state failed: {ik_error}')
            return None

        for attempt in range(2):
            goal = self._build_joint_goal(joint_target, plan_only=True)
            send_future = self._move_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(
                self, send_future, timeout_sec=self._motion_timeout_sec)
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warn(
                    f'MoveGroup goal rejected or timed out (attempt {attempt + 1}/2)')
                time.sleep(0.5)
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=self._motion_timeout_sec)
            result = result_future.result()
            if result is None:
                self.get_logger().warn(
                    f'MoveGroup planning timed out (attempt {attempt + 1}/2)')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
                continue
            error_code = result.result.error_code.val
            if error_code == 1:  # moveit_msgs/MoveItErrorCodes.SUCCESS
                trajectory = result.result.planned_trajectory
                if not trajectory.joint_trajectory.points:
                    self.get_logger().warn('MoveGroup returned an empty planned trajectory')
                    continue
                return trajectory
            self.get_logger().warn(
                f'MoveGroup planning failed with error_code={error_code} '
                f'(attempt {attempt + 1}/2)')
            time.sleep(0.5)
        return None

    def _trajectory_positions(self, trajectory):
        joint_trajectory = trajectory.joint_trajectory
        try:
            indices = [joint_trajectory.joint_names.index(name) for name in self._joint_names]
        except ValueError as ex:
            raise ValueError(f'planned trajectory is missing a required joint: {ex}') from ex
        positions = []
        for point in joint_trajectory.points:
            if any(index >= len(point.positions) for index in indices):
                raise ValueError('planned trajectory point has incomplete positions')
            positions.append([point.positions[index] for index in indices])
        return np.asarray(positions, dtype=float)

    @staticmethod
    def _trajectory_duration_sec(trajectory):
        point = trajectory.joint_trajectory.points[-1]
        return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

    def _validate_trajectory_joint_motion(self, trajectory, start_positions):
        try:
            planned_positions = self._trajectory_positions(trajectory)
        except ValueError as ex:
            return False, str(ex)
        max_delta = np.max(np.abs(planned_positions - start_positions), axis=0)
        violations = [
            f'{name}={delta:.3f}>{limit:.3f}rad'
            for name, delta, limit in zip(
                self._joint_names, max_delta, self._max_joint_delta)
            if delta > limit + 1e-9
        ]
        if violations:
            return False, 'excessive joint motion: ' + ', '.join(violations)
        return True, (
            'max joint deltas=' +
            ', '.join(f'{name}:{delta:.3f}' for name, delta in zip(self._joint_names, max_delta)))

    def _check_target_link_visibility(self, trajectory):
        if not self._visibility_links:
            return True, 'link visibility check disabled'
        if not self._fk_client.wait_for_service(timeout_sec=3.0):
            return False, f'FK service {self._fk_service_name} is unavailable'

        final_positions = self._trajectory_positions(trajectory)[-1]
        request = GetPositionFK.Request()
        request.header.frame_id = self._base_frame
        request.fk_link_names = self._visibility_links
        request.robot_state = RobotState()
        request.robot_state.joint_state.name = self._joint_names
        request.robot_state.joint_state.position = final_positions.tolist()
        request.robot_state.is_diff = True

        future = self._fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        response = future.result()
        if response is None:
            return False, 'FK request timed out'
        if response.error_code.val != 1:
            return False, f'FK failed with error_code={response.error_code.val}'

        outside = []
        for link_name, pose_stamped in zip(response.fk_link_names, response.pose_stamped):
            p = pose_stamped.pose.position
            xyz = np.array([p.x, p.y, p.z], dtype=float)
            if not self._in_camera_fov(xyz, margin_rad=self._visibility_margin_rad):
                outside.append(link_name)
        if outside:
            return False, (
                'required links outside the approximate camera safe area: ' +
                ', '.join(outside))
        return True, 'required forearm/wrist/tool0 link origins inside camera safe area'

    def _execute_planned_trajectory(self, trajectory):
        if not self._execute_client.wait_for_server(timeout_sec=self._motion_timeout_sec):
            self.get_logger().error('/execute_trajectory action server not available')
            return False

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self._execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=self._motion_timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('ExecuteTrajectory goal rejected or timed out')
            return False

        timeout = max(
            self._motion_timeout_sec,
            self._trajectory_duration_sec(trajectory) + self._settle_timeout + 5.0)
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        result = result_future.result()
        if result is None:
            self.get_logger().error('ExecuteTrajectory timed out; requesting cancel')
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            return False
        if result.result.error_code.val != 1:
            self.get_logger().error(
                f'ExecuteTrajectory failed with error_code={result.result.error_code.val}')
            return False
        return True

    def _wait_until_still(self):
        deadline = time.monotonic() + self._settle_timeout
        stable_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            _, velocities, stamp = self._get_joint_state()
            if velocities is None or stamp is None or time.monotonic() - stamp > 0.5:
                stable_since = None
                continue
            if float(np.max(np.abs(velocities))) <= self._settle_velocity_threshold:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= self._settle_stable_time:
                    if self._settle_time_sec > 0.0:
                        time.sleep(self._settle_time_sec)
                    return True
            else:
                stable_since = None
        self.get_logger().warn(
            f'Robot did not remain below {self._settle_velocity_threshold:.4f}rad/s '
            f'for {self._settle_stable_time:.2f}s within the settle timeout')
        return False

    def _lookup_calibration_point_base(self):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                t = self._tf_buffer.lookup_transform(
                    self._base_frame, self._calibration_frame, rclpy.time.Time())
                p = t.transform.translation
                return Point(x=p.x, y=p.y, z=p.z)
            except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().warn(
                    f'tf2 lookup {self._base_frame}->{self._calibration_frame} failed: {ex}')
                rclpy.spin_once(self, timeout_sec=0.2)
        return None

    def _on_capture_done(self, msg: CaptureDone):
        if self._capture_done_event is not None and msg.pose_index == self._capture_done_event:
            self._capture_done_success = msg.success
            self._capture_done_event = None

    def _wait_capture_done(self, pose_index: int) -> bool:
        self._capture_done_event = pose_index
        self._capture_done_success = False
        deadline = time.monotonic() + self._capture_timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._capture_done_event is None:
                return self._capture_done_success
        self.get_logger().warn(
            f'capture_done timeout for pose {pose_index}; proceeding to next pose anyway')
        self._capture_done_event = None
        return False

    def run(self):
        poses = self.generate_poses()
        self.get_logger().info(
            f'Generated {len(poses)} structured poses '
            f'(validate={self._validate}, plan_only={self._plan_only}, step={self._step})')
        if self._dry_run:
            previous = self._sampling_center
            for i, (xyz, rpy) in enumerate(poses):
                step = float(np.linalg.norm(xyz - previous))
                print(f'[{i}] xyz={xyz.tolist()} rpy={rpy} step_from_previous={step:.3f}m')
                previous = xyz
            return

        for i, (xyz, rpy) in enumerate(poses):
            start_positions = self._wait_for_joint_state()
            if start_positions is None:
                self.get_logger().error(
                    f'Pose {i}: no fresh {self._joint_states_topic}; aborting the sequence')
                break

            self.get_logger().info(f'Pose {i}: planning xyz={xyz.tolist()} rpy={rpy}')
            try:
                trajectory = self._plan_pose(xyz, rpy, start_positions)
            except Exception as ex:  # noqa: BLE001 - never abort the whole sequence
                self.get_logger().error(f'Pose {i}: planning raised {ex!r}; skipping')
                continue
            if trajectory is None:
                self.get_logger().warn(f'Pose {i}: planning failed after retry; skipping')
                continue

            joint_ok, joint_report = self._validate_trajectory_joint_motion(
                trajectory, start_positions)
            if not joint_ok:
                self.get_logger().warn(f'Pose {i}: rejected plan: {joint_report}')
                continue

            try:
                visibility_ok, visibility_report = self._check_target_link_visibility(trajectory)
            except Exception as ex:  # noqa: BLE001 - failed checks must reject, never execute
                self.get_logger().error(
                    f'Pose {i}: visibility check raised {ex!r}; rejecting plan')
                continue
            if not visibility_ok:
                self.get_logger().warn(f'Pose {i}: rejected plan: {visibility_report}')
                continue

            duration = self._trajectory_duration_sec(trajectory)
            self.get_logger().info(
                f'Pose {i}: plan accepted, duration={duration:.2f}s; '
                f'{joint_report}; {visibility_report}')

            if self._plan_only:
                continue

            if self._step:
                answer = input(
                    f'Execute pose {i} ({duration:.2f}s)? [y]es/[s]kip/[q]uit: '
                ).strip().lower()
                if answer == 'q':
                    self.get_logger().info('Operator stopped the pose sequence')
                    break
                if answer not in ('y', 'yes'):
                    self.get_logger().info(f'Pose {i}: skipped by operator')
                    continue

            if not self._execute_planned_trajectory(trajectory):
                self.get_logger().warn(f'Pose {i}: execution failed; skipping capture')
                continue
            if not self._wait_until_still():
                self.get_logger().warn(f'Pose {i}: robot did not settle; skipping capture')
                continue

            p_base = self._lookup_calibration_point_base()
            if p_base is None:
                self.get_logger().warn(
                    f'Pose {i}: could not get {self._calibration_frame} p_base via tf2; skipping')
                continue

            msg = PoseReady()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._base_frame
            msg.pose_index = i
            msg.p_base = p_base
            msg.is_validation = self._validate
            self._pose_ready_pub.publish(msg)
            self.get_logger().info(
                f'Pose {i}: published pose_ready {self._calibration_frame} '
                f'p_base=({p_base.x:.4f},{p_base.y:.4f},{p_base.z:.4f})')

            captured = self._wait_capture_done(i)
            self.get_logger().info(f'Pose {i}: capture_done success={captured}')

        self.get_logger().info('Pose sequence complete.')


def main(args=None):
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--dry-run', action='store_true')
    mode.add_argument('--plan-only', action='store_true')
    parser.add_argument('--step', action='store_true')
    parser.add_argument('--validate', action='store_true')
    parsed, ros_args = parser.parse_known_args(sys.argv[1:])

    if parsed.plan_only and parsed.step:
        parser.error('--step cannot be combined with --plan-only')

    rclpy.init(args=ros_args)
    node = PoseSamplerNode(
        dry_run=parsed.dry_run,
        plan_only=parsed.plan_only,
        step=parsed.step,
        validate=parsed.validate)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
