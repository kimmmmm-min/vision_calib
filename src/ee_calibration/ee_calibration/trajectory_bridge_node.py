#!/usr/bin/env python3
"""FollowJointTrajectory action server that bridges MoveIt2's execution layer
to urcb2_driver's realtime servoj streaming interface.

ur_moveit_config's config/controllers.yaml declares
`scaled_joint_trajectory_controller` (action_ns follow_joint_trajectory, type
FollowJointTrajectory) as the default controller for the ur_manipulator
group. moveit_simple_controller_manager therefore looks for an action server
named exactly '/scaled_joint_trajectory_controller/follow_joint_trajectory' --
this node *is* that server, so no custom MoveIt2 controller config is needed.

urcb2_driver's commandLoop() (yur_ros_wrapper.cpp) reads whatever was last
published to /UR10_right/targetJ every 8ms and issues servoj() with it; if no
message has arrived in the last 100ms it auto-stops (stopj). This node must
therefore keep streaming fresh targets throughout trajectory execution, at a
rate comfortably under that 100ms budget -- default 50 Hz (20ms), a 5x margin.
This node must run on Desktop (co-located with urcb2_driver), never on the
Jetson, because the stream is latency-sensitive.
"""
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

CANONICAL_JOINT_ORDER = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


def _hermite_interp(p0, v0, p1, v1, t, T):
    """Cubic Hermite interpolation of a single joint between two waypoints.

    p0/v0 at t=0, p1/v1 at t=T. Falls back to linear if T <= 0.
    """
    if T <= 1e-6:
        return p1
    s = t / T
    s2 = s * s
    s3 = s2 * s
    h00 = 2 * s3 - 3 * s2 + 1
    h10 = s3 - 2 * s2 + s
    h01 = -2 * s3 + 3 * s2
    h11 = s3 - s2
    return h00 * p0 + h10 * T * v0 + h01 * p1 + h11 * T * v1


class TrajectoryBridgeNode(Node):

    def __init__(self):
        super().__init__('trajectory_bridge_node')

        self.declare_parameter('action_name',
                                '/scaled_joint_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('target_topic', '/UR10_right/targetJ')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('stream_rate_hz', 50.0)
        self.declare_parameter('goal_tolerance_rad', 0.01)
        self.declare_parameter('settle_check_timeout_sec', 2.0)
        self.declare_parameter('canonical_joint_order', CANONICAL_JOINT_ORDER)

        self._action_name = self.get_parameter('action_name').value
        self._target_topic = self.get_parameter('target_topic').value
        self._joint_states_topic = self.get_parameter('joint_states_topic').value
        self._stream_rate_hz = float(self.get_parameter('stream_rate_hz').value)
        self._goal_tolerance_rad = float(self.get_parameter('goal_tolerance_rad').value)
        self._settle_check_timeout_sec = float(
            self.get_parameter('settle_check_timeout_sec').value)
        self._canonical_order = list(self.get_parameter('canonical_joint_order').value)

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.RELIABLE
        sensor_qos.history = HistoryPolicy.KEEP_LAST

        self._target_pub = self.create_publisher(
            Float64MultiArray, self._target_topic, sensor_qos)

        self._latest_positions = None
        self._latest_positions_lock = threading.Lock()
        self._joint_states_sub = self.create_subscription(
            JointState, self._joint_states_topic, self._on_joint_states, sensor_qos)

        self._cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self._action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f'trajectory_bridge_node serving {self._action_name} -> {self._target_topic} '
            f'at {self._stream_rate_hz} Hz')

    def _on_joint_states(self, msg: JointState):
        if not msg.name:
            return
        try:
            ordered = [msg.position[msg.name.index(j)] for j in self._canonical_order]
        except ValueError:
            return
        with self._latest_positions_lock:
            self._latest_positions = ordered

    def _get_latest_positions(self):
        with self._latest_positions_lock:
            return None if self._latest_positions is None else list(self._latest_positions)

    def _goal_callback(self, goal_request):
        if not goal_request.trajectory.points:
            self.get_logger().warn('Rejecting empty trajectory goal')
            return GoalResponse.REJECT
        for name in self._canonical_order:
            if name not in goal_request.trajectory.joint_names:
                self.get_logger().warn(
                    f'Rejecting goal: joint "{name}" missing from trajectory.joint_names')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _reindex_point(self, joint_names, point):
        idx = [joint_names.index(j) for j in self._canonical_order]
        positions = np.array([point.positions[i] for i in idx], dtype=float)
        if point.velocities:
            velocities = np.array([point.velocities[i] for i in idx], dtype=float)
        else:
            velocities = np.zeros(6)
        t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        return t, positions, velocities

    def _execute_callback(self, goal_handle):
        traj = goal_handle.request.trajectory
        joint_names = list(traj.joint_names)
        waypoints = [self._reindex_point(joint_names, p) for p in traj.points]
        waypoints.sort(key=lambda w: w[0])

        result = FollowJointTrajectory.Result()
        period = 1.0 / self._stream_rate_hz
        start_positions = self._get_latest_positions()
        if start_positions is None:
            self.get_logger().error(
                'No /joint_states received yet; aborting trajectory (bridge or '
                'joint_state_bridge_node not running?)')
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'no current joint state available'
            return result

        segs = [(0.0, np.array(start_positions), np.zeros(6))] + waypoints
        total_time = segs[-1][0]

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = self._canonical_order

        t0 = time.monotonic()
        canceled = False
        while True:
            elapsed = time.monotonic() - t0
            if goal_handle.is_cancel_requested:
                canceled = True
                break
            if elapsed >= total_time:
                target = segs[-1][1]
                self._publish_target(target)
                break

            seg_idx = 0
            for i in range(len(segs) - 1):
                if segs[i][0] <= elapsed <= segs[i + 1][0]:
                    seg_idx = i
                    break
            t_a, p_a, v_a = segs[seg_idx]
            t_b, p_b, v_b = segs[seg_idx + 1]
            local_t = elapsed - t_a
            local_T = t_b - t_a
            target = np.array([
                _hermite_interp(p_a[j], v_a[j], p_b[j], v_b[j], local_t, local_T)
                for j in range(6)
            ])
            self._publish_target(target)

            feedback.desired.positions = target.tolist()
            actual = self._get_latest_positions()
            if actual is not None:
                feedback.actual.positions = actual
            goal_handle.publish_feedback(feedback)

            time.sleep(period)

        if canceled:
            goal_handle.canceled()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = 'canceled by client'
            self.get_logger().info('Trajectory canceled; urcb2_driver will stopj on stale stream')
            return result

        final_target = segs[-1][1]
        settle_deadline = time.monotonic() + self._settle_check_timeout_sec
        reached = False
        while time.monotonic() < settle_deadline:
            self._publish_target(final_target)
            actual = self._get_latest_positions()
            if actual is not None:
                err = np.max(np.abs(np.array(actual) - final_target))
                if err <= self._goal_tolerance_rad:
                    reached = True
                    break
            time.sleep(period)

        if reached:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        else:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = (
                f'final position not within {self._goal_tolerance_rad} rad after '
                f'{self._settle_check_timeout_sec}s settle window')
        return result

    def _publish_target(self, positions):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in positions]
        self._target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
