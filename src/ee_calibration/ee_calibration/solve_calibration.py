#!/usr/bin/env python3
"""Jetson script: solves T_base<-camera from a manifest of (p_base, pixel)
correspondences using cv2.solvePnP, with an explicit numeric PASS/FAIL
reprojection-error check (never a qualitative judgment call).

Usage:
    solve_calibration --manifest ~/calib_ws/calib_data/<run>/manifest.json \
        [--method ITERATIVE] [--max-reprojection-error-px 5.0] \
        [--cross-check-3d3d] [--validate] [--out t_base_camera.yaml]

max_reprojection_error_px default derivation (see README for the full
rationale): hysteresis_margin=0.03m and measured color-camera intrinsics
fx=605.74 fy=605.55 (matches project's fx=~605.6 assumption at 640x480):
    position_error(m) ~= (reprojection_error_px / fx) * operating_distance_m
At operating_distance=1.0m, 5px ~= 8.3mm ~= 28% of the 30mm hysteresis margin.
"""
import argparse
import json
import os

import cv2
import numpy as np

PNP_METHODS = {
    'ITERATIVE': cv2.SOLVEPNP_ITERATIVE,
    'EPNP': cv2.SOLVEPNP_EPNP,
    'SQPNP': cv2.SOLVEPNP_SQPNP,
    'P3P': cv2.SOLVEPNP_P3P,
}


def _load_manifest(path):
    with open(path) as f:
        return json.load(f)


def _camera_matrix(camera_info):
    k = camera_info['k']
    K = np.array(k, dtype=float).reshape(3, 3)
    D = np.array(camera_info['d'], dtype=float)
    return K, D


def _rotmat_to_quat(R):
    """Manual rotation-matrix -> quaternion (w,x,y,z), no scipy/tf_transformations dep."""
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    tr = m00 + m11 + m22
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        w = 0.25 * S
        x = (m21 - m12) / S
        y = (m02 - m20) / S
        z = (m10 - m01) / S
    elif m00 > m11 and m00 > m22:
        S = np.sqrt(1.0 + m00 - m11 - m22) * 2
        w = (m21 - m12) / S
        x = 0.25 * S
        y = (m01 + m10) / S
        z = (m02 + m20) / S
    elif m11 > m22:
        S = np.sqrt(1.0 + m11 - m00 - m22) * 2
        w = (m02 - m20) / S
        x = (m01 + m10) / S
        y = 0.25 * S
        z = (m12 + m21) / S
    else:
        S = np.sqrt(1.0 + m22 - m00 - m11) * 2
        w = (m10 - m01) / S
        x = (m02 + m20) / S
        y = (m12 + m21) / S
        z = 0.25 * S
    return np.array([x, y, z, w])  # geometry_msgs order (x,y,z,w)


def _kabsch_umeyama(src_pts, dst_pts):
    """Rigid (no-scale) transform mapping src_pts -> dst_pts via SVD.

    Returns (R, t) such that dst ~= R @ src + t.
    """
    src_mean = src_pts.mean(axis=0)
    dst_mean = dst_pts.mean(axis=0)
    src_c = src_pts - src_mean
    dst_c = dst_pts - dst_mean
    H = src_c.T @ dst_c
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    t = dst_mean - R @ src_mean
    return R, t


def _depth_backproject(depth_img, u, v, K, depth_scale):
    """Same manual pinhole back-projection style as hand_detector_node.py."""
    z = float(depth_img[v, u]) * depth_scale
    if z <= 0:
        return None
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.array([x, y, z])


def _collect_correspondences(manifest, want_validation: bool):
    entries = [
        e for e in manifest['entries']
        if e.get('success') and e.get('click_status') == 'confirmed'
        and e.get('pixel_uv') is not None
        and bool(e.get('is_validation', False)) == want_validation
    ]
    p_base = np.array([[e['p_base']['x'], e['p_base']['y'], e['p_base']['z']] for e in entries])
    pixels = np.array([e['pixel_uv'] for e in entries], dtype=float)
    return entries, p_base, pixels


def _reprojection_errors(p_base, pixels, rvec, tvec, K, D):
    projected, _ = cv2.projectPoints(p_base, rvec, tvec, K, D)
    projected = projected.reshape(-1, 2)
    errors = np.linalg.norm(projected - pixels, axis=1)
    return errors, projected


def solve(manifest, method: str):
    K, D = _camera_matrix(manifest['camera_info'])
    entries, p_base, pixels = _collect_correspondences(manifest, want_validation=False)
    if len(entries) < 4:
        raise RuntimeError(
            f'Need >= 4 confirmed training correspondences for solvePnP, got {len(entries)}')

    ok, rvec, tvec = cv2.solvePnP(
        p_base.astype(np.float64), pixels.astype(np.float64), K, D,
        flags=PNP_METHODS[method])
    if not ok:
        raise RuntimeError('cv2.solvePnP failed to converge')

    R_cam_base, _ = cv2.Rodrigues(rvec)
    T_cam_base = np.eye(4)
    T_cam_base[:3, :3] = R_cam_base
    T_cam_base[:3, 3] = tvec.flatten()
    T_base_cam = np.linalg.inv(T_cam_base)

    return {
        'entries': entries, 'p_base': p_base, 'pixels': pixels,
        'rvec': rvec, 'tvec': tvec, 'K': K, 'D': D,
        'T_cam_base': T_cam_base, 'T_base_cam': T_base_cam,
    }


def report_reprojection(entries, p_base, pixels, rvec, tvec, K, D, max_err_px, label):
    errors, projected = _reprojection_errors(p_base, pixels, rvec, tvec, K, D)
    rms = float(np.sqrt(np.mean(errors ** 2)))
    passed = rms <= max_err_px

    print(f'\n=== Reprojection error report [{label}] ===')
    print(f'N poses: {len(entries)}')
    print(f'RMS reprojection error: {rms:.3f} px  (threshold: {max_err_px:.3f} px)')
    print(f'RESULT: {"PASS" if passed else "FAIL"}')

    order = np.argsort(-errors)
    print('Worst poses (candidates for re-clicking):')
    for idx in order[:min(5, len(order))]:
        e = entries[idx]
        print(f'  pose {e["pose_index"]:>3}  error={errors[idx]:6.2f}px  '
              f'p_base=({e["p_base"]["x"]:.3f},{e["p_base"]["y"]:.3f},{e["p_base"]["z"]:.3f})')

    bbox_min = p_base.min(axis=0)
    bbox_max = p_base.max(axis=0)
    print(f'3D spread of this set: min={bbox_min.tolist()} max={bbox_max.tolist()} '
          f'(depth range hint -- narrow spread degrades PnP conditioning)')

    return rms, passed, errors


def cross_check_3d3d(manifest, entries, p_base):
    K, _ = _camera_matrix(manifest['camera_info'])
    depth_scale = 0.001  # 16UC1 mm -> m, same convention as hand_detector_node.py

    p_cam_list = []
    p_base_list = []
    for e, pb in zip(entries, p_base):
        if not e.get('depth_path') or not os.path.exists(e['depth_path']):
            continue
        depth_img = cv2.imread(e['depth_path'], cv2.IMREAD_UNCHANGED)
        if depth_img is None:
            continue
        u, v = e['pixel_uv']
        p_cam = _depth_backproject(depth_img, int(round(u)), int(round(v)), K, depth_scale)
        if p_cam is None:
            continue
        p_cam_list.append(p_cam)
        p_base_list.append(pb)

    if len(p_cam_list) < 4:
        print(f'\n=== 3D-3D cross-check === skipped: only {len(p_cam_list)} usable '
              f'depth points (need >= 4)')
        return

    p_cam_arr = np.array(p_cam_list)
    p_base_arr = np.array(p_base_list)
    R, t = _kabsch_umeyama(p_cam_arr, p_base_arr)  # maps p_cam -> p_base i.e. R,t of T_base_cam
    T_base_cam_3d3d = np.eye(4)
    T_base_cam_3d3d[:3, :3] = R
    T_base_cam_3d3d[:3, 3] = t

    residuals = np.linalg.norm((R @ p_cam_arr.T).T + t - p_base_arr, axis=1)
    print(f'\n=== 3D-3D cross-check (independent estimate, N={len(p_cam_list)}) ===')
    print(f'RMS 3D residual: {np.sqrt(np.mean(residuals ** 2)) * 1000:.2f} mm')
    print('T_base<-camera (3D-3D):')
    print(T_base_cam_3d3d)
    print('(printed side-by-side for comparison only -- not auto-merged with the PnP result)')


def write_outputs(T_base_cam, out_path, base_frame, camera_frame):
    quat = _rotmat_to_quat(T_base_cam[:3, :3])
    trans = T_base_cam[:3, 3]

    yaml_content = (
        f'# Generated by solve_calibration.py -- T_{base_frame}<-{camera_frame}\n'
        f'translation:\n'
        f'  x: {trans[0]:.6f}\n  y: {trans[1]:.6f}\n  z: {trans[2]:.6f}\n'
        f'rotation:\n'
        f'  x: {quat[0]:.6f}\n  y: {quat[1]:.6f}\n  z: {quat[2]:.6f}\n  w: {quat[3]:.6f}\n'
    )
    with open(out_path, 'w') as f:
        f.write(yaml_content)

    launch_path = out_path.rsplit('.', 1)[0] + '_static_tf.launch.py'
    launch_content = f'''from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ee_calibration_static_tf',
            arguments=[
                '{trans[0]:.6f}', '{trans[1]:.6f}', '{trans[2]:.6f}',
                '{quat[0]:.6f}', '{quat[1]:.6f}', '{quat[2]:.6f}', '{quat[3]:.6f}',
                '{base_frame}', '{camera_frame}',
            ],
        ),
    ])
'''
    with open(launch_path, 'w') as f:
        f.write(launch_content)

    print(f'\nWrote {out_path} and {launch_path}')
    print('\nT_base<-camera (4x4):')
    print(T_base_cam)
    print(f'translation (m): {trans.tolist()}')
    print(f'quaternion (x,y,z,w): {quat.tolist()}')
    cmd = (
        f"ros2 run tf2_ros static_transform_publisher "
        f"{trans[0]:.6f} {trans[1]:.6f} {trans[2]:.6f} "
        f"{quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f} "
        f"{base_frame} {camera_frame}"
    )
    print(f'\nOne-off command:\n  {cmd}')


def run_validate(manifest, T_base_cam, K, D, max_err_px):
    entries, p_base, pixels = _collect_correspondences(manifest, want_validation=True)
    if len(entries) == 0:
        print('\n=== Validation === no is_validation=True confirmed entries in manifest; '
              're-run pose_sampler_node --validate on Desktop first')
        return
    T_cam_base = np.linalg.inv(T_base_cam)
    rvec, _ = cv2.Rodrigues(T_cam_base[:3, :3])
    tvec = T_cam_base[:3, 3].reshape(3, 1)
    errors, _ = _reprojection_errors(p_base, pixels, rvec, tvec, K, D)

    print(f'\n=== Validation (new poses, not used in solving) ===')
    overall_pass = True
    for e, err in zip(entries, errors):
        passed = err <= max_err_px
        overall_pass = overall_pass and passed
        print(f'  pose {e["pose_index"]:>3}  error={err:6.2f}px  '
              f'{"PASS" if passed else "FAIL"}')
    rms = float(np.sqrt(np.mean(errors ** 2)))
    print(f'Validation RMS: {rms:.3f}px (threshold {max_err_px:.3f}px)')
    print(f'OVERALL VALIDATION RESULT: {"PASS" if overall_pass and rms <= max_err_px else "FAIL"}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--method', default='ITERATIVE', choices=list(PNP_METHODS.keys()))
    parser.add_argument('--max-reprojection-error-px', type=float, default=5.0)
    parser.add_argument('--cross-check-3d3d', action='store_true')
    parser.add_argument('--validate', action='store_true')
    parser.add_argument('--out', default=None)
    parser.add_argument('--base-frame', default='base')
    parser.add_argument('--camera-frame', default='camera_color_optical_frame')
    args = parser.parse_args()

    manifest_path = os.path.expanduser(args.manifest)
    manifest = _load_manifest(manifest_path)
    if manifest.get('camera_info') is None:
        raise RuntimeError('manifest has no cached camera_info; run image_capture_node first')

    solved = solve(manifest, args.method)
    report_reprojection(
        solved['entries'], solved['p_base'], solved['pixels'],
        solved['rvec'], solved['tvec'], solved['K'], solved['D'],
        args.max_reprojection_error_px, label='training set')

    if args.cross_check_3d3d:
        cross_check_3d3d(manifest, solved['entries'], solved['p_base'])

    out_path = args.out or os.path.join(os.path.dirname(manifest_path), 't_base_camera.yaml')
    write_outputs(solved['T_base_cam'], out_path, args.base_frame, args.camera_frame)

    if args.validate:
        run_validate(
            manifest, solved['T_base_cam'], solved['K'], solved['D'],
            args.max_reprojection_error_px)


if __name__ == '__main__':
    main()
