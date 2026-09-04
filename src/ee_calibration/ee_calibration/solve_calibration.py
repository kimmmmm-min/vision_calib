#!/usr/bin/env python3
"""Jetson script: solves T_base<-camera from a manifest of (p_base, pixel,
depth frame) correspondences, with an explicit numeric PASS/FAIL error
check (never a qualitative judgment call).

Default method (--method FUSED) is a true 2D+3D sensor fusion: one joint
nonlinear least-squares optimization whose cost function combines the
pixel reprojection residual (2D, from the clicked/detected pixel) AND the
depth-backprojected 3D residual (from the depth frame at that same pixel)
for every correspondence, weighted by their respective assumed measurement
noise (see solve_fused()). This is not "depth primary, PnP as a fallback/
cross-check" -- both signals shape the same solve simultaneously. It is
initialized from the depth-only Kabsch/Umeyama solve (--method
KABSCH_3D3D), which is available on its own, and classic 2D-only solvePnP
is still available via --method ITERATIVE/EPNP/SQPNP/P3P.

Usage:
    solve_calibration --manifest ~/calib_ws/calib_data/<run>/manifest.json \
        [--method FUSED] [--sigma-2d-px 1.5] [--sigma-3d-mm 8.0] \
        [--max-reprojection-error-px 5.0] [--max-3d-error-mm 10.0] \
        [--cross-check-3d3d] [--validate] [--out t_base_camera.yaml]

max_reprojection_error_px default derivation (see README for the full
rationale): hysteresis_margin=0.03m and measured color-camera intrinsics
fx=605.74 fy=605.55 (matches project's fx=~605.6 assumption at 640x480):
    position_error(m) ~= (reprojection_error_px / fx) * operating_distance_m
At operating_distance=1.0m, 5px ~= 8.3mm ~= 28% of the 30mm hysteresis margin.
2026-09-04: max_3d_error_mm tightened 30.0 -> 10.0mm per user request (the
30mm hysteresis-margin-derived value was a loose default, not a hard
requirement; 10mm is a stricter pass bar for 3D validation).
"""
import argparse
import json
import os

import cv2
import numpy as np
from scipy.optimize import least_squares

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


def _backproject_entries(entries, p_base, K, depth_scale=0.001):
    """Back-project each entry's clicked pixel through its captured depth
    frame into camera-frame 3D. Returns (entries_used, p_cam_arr, p_base_arr,
    pixels_used) -- entries without a usable depth reading are dropped.
    """
    p_cam_list, p_base_list, pixels_list, entries_used = [], [], [], []
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
        pixels_list.append(e['pixel_uv'])
        entries_used.append(e)
    return (entries_used, np.array(p_cam_list) if p_cam_list else np.empty((0, 3)),
            np.array(p_base_list) if p_base_list else np.empty((0, 3)),
            np.array(pixels_list) if pixels_list else np.empty((0, 2)))


def solve_3d3d(manifest, entries, p_base, K):
    """Depth-aware calibration: back-project each clicked pixel through its
    depth frame to a camera-frame 3D point, then solve the rigid transform
    (Kabsch/Umeyama, SVD) mapping camera-frame points -> base-frame points.
    Unlike solvePnP (2D-3D, ignores depth entirely), this uses the actual
    measured depth at every correspondence.
    """
    entries_used, p_cam_arr, p_base_arr, pixels_used = _backproject_entries(entries, p_base, K)
    if len(entries_used) < 4:
        raise RuntimeError(
            f'Need >= 4 confirmed training correspondences with usable depth for '
            f'3D-3D solve, got {len(entries_used)} (out of {len(entries)} confirmed)')

    R, t = _kabsch_umeyama(p_cam_arr, p_base_arr)  # maps p_cam -> p_base i.e. R,t of T_base_cam
    T_base_cam = np.eye(4)
    T_base_cam[:3, :3] = R
    T_base_cam[:3, 3] = t
    T_cam_base = np.linalg.inv(T_base_cam)

    residuals = np.linalg.norm((R @ p_cam_arr.T).T + t - p_base_arr, axis=1)
    rms_3d_mm = float(np.sqrt(np.mean(residuals ** 2)) * 1000)
    print(f'\n=== 3D-3D (depth-based) solve ===')
    print(f'N points with usable depth: {len(entries_used)} / {len(entries)} confirmed training entries')
    print(f'RMS 3D residual: {rms_3d_mm:.2f} mm')

    rvec, _ = cv2.Rodrigues(T_cam_base[:3, :3])
    tvec = T_cam_base[:3, 3].reshape(3, 1)

    return {
        'entries': entries_used, 'p_base': p_base_arr, 'pixels': pixels_used,
        'rvec': rvec, 'tvec': tvec,
        'T_cam_base': T_cam_base, 'T_base_cam': T_base_cam,
        'rms_3d_mm': rms_3d_mm,
    }


def solve_fused(manifest, entries, p_base, K, D, sigma_2d_px=1.5, sigma_3d_mm=8.0):
    """True 2D+3D fusion: one joint nonlinear least-squares refinement of
    T_cam_base that minimizes normalized reprojection error (pixel, the
    solvePnP-style geometric residual) AND normalized depth-backprojected
    3D error (mm) TOGETHER in a single cost function -- not one method
    solved and the other used only for cross-check/validation.

    Every confirmed correspondence contributes its 2D pixel residual.
    Correspondences with a usable depth frame additionally contribute a 3D
    residual. The two residual types are on different units/noise scales,
    so each is divided by an assumed 1-sigma measurement noise
    (sigma_2d_px for click/detection pixel noise, sigma_3d_mm for the
    RealSense depth noise at ~1m operating distance) before being stacked
    into one residual vector -- this is a standard weighted-least-squares
    sensor fusion: it's equivalent to a joint Gaussian MLE over both
    sensors assuming those noise levels.

    Initialized from the Kabsch 3D-3D solve on the depth-usable subset,
    which is normally already close, then refined by Levenberg-Marquardt.
    """
    entries_d, p_cam_arr, p_base_d, pixels_d = _backproject_entries(entries, p_base, K)
    if len(entries_d) < 4:
        raise RuntimeError(
            f'Need >= 4 confirmed training correspondences with usable depth for '
            f'fused solve, got {len(entries_d)} (out of {len(entries)} confirmed)')

    depth_lookup = {id(e): pc for e, pc in zip(entries_d, p_cam_arr)}

    R0, t0 = _kabsch_umeyama(p_cam_arr, p_base_d)
    T_base_cam0 = np.eye(4)
    T_base_cam0[:3, :3] = R0
    T_base_cam0[:3, 3] = t0
    T_cam_base0 = np.linalg.inv(T_base_cam0)
    rvec0, _ = cv2.Rodrigues(T_cam_base0[:3, :3])
    x0 = np.concatenate([rvec0.flatten(), T_cam_base0[:3, 3].flatten()])

    pixels_all = np.array([e['pixel_uv'] for e in entries], dtype=float)
    depth_pc = [depth_lookup.get(id(e)) for e in entries]

    def residuals(x):
        rvec = x[:3].reshape(3, 1)
        tvec = x[3:6].reshape(3, 1)
        R_cam_base, _ = cv2.Rodrigues(rvec)
        R_base_cam = R_cam_base.T
        t_base_cam = -R_base_cam @ tvec.flatten()

        proj, _ = cv2.projectPoints(p_base, rvec, tvec, K, D)
        proj = proj.reshape(-1, 2)
        res_2d = (proj - pixels_all) / sigma_2d_px

        res_3d = [
            (R_base_cam @ pc + t_base_cam - pb) * 1000.0 / sigma_3d_mm
            for pc, pb in zip(depth_pc, p_base) if pc is not None
        ]
        res_3d = np.array(res_3d) if res_3d else np.empty((0, 3))
        return np.concatenate([res_2d.flatten(), res_3d.flatten()])

    result = least_squares(residuals, x0, method='lm')
    rvec = result.x[:3].reshape(3, 1)
    tvec = result.x[3:6].reshape(3, 1)
    R_cam_base, _ = cv2.Rodrigues(rvec)
    T_cam_base = np.eye(4)
    T_cam_base[:3, :3] = R_cam_base
    T_cam_base[:3, 3] = tvec.flatten()
    T_base_cam = np.linalg.inv(T_cam_base)

    errors_2d, _ = _reprojection_errors(p_base, pixels_all, rvec, tvec, K, D)
    rms_2d = float(np.sqrt(np.mean(errors_2d ** 2)))

    R_base_cam, t_base_cam = T_base_cam[:3, :3], T_base_cam[:3, 3]
    pred_3d = (R_base_cam @ p_cam_arr.T).T + t_base_cam
    dist_3d_mm = np.linalg.norm(pred_3d - p_base_d, axis=1) * 1000
    rms_3d_mm = float(np.sqrt(np.mean(dist_3d_mm ** 2)))

    print(f'\n=== Fused 2D+3D solve (joint nonlinear least squares) ===')
    print(f'N correspondences: {len(entries)} total, {len(entries_d)} with usable depth')
    print(f'weights: sigma_2d={sigma_2d_px:.2f}px  sigma_3d={sigma_3d_mm:.2f}mm')
    print(f'RMS reprojection error @ fused solution: {rms_2d:.3f} px')
    print(f'RMS 3D (depth) error   @ fused solution: {rms_3d_mm:.2f} mm')
    print(f'optimizer: status={result.status} cost={result.cost:.4f} nfev={result.nfev}')

    return {
        'entries': entries, 'p_base': p_base, 'pixels': pixels_all,
        'rvec': rvec, 'tvec': tvec,
        'T_cam_base': T_cam_base, 'T_base_cam': T_base_cam,
        'rms_2d_px': rms_2d, 'rms_3d_mm': rms_3d_mm,
    }


def solve_hybrid_2d3d(manifest, entries, p_base, K, D, *,
                       sigma_px=1.5, depth_rel_error=0.02, min_depth_sigma_m=0.005,
                       depth_scale=0.001):
    """Uncertainty-weighted joint 2D+3D fusion, additive alongside (not a
    replacement for) --method KABSCH_3D3D / PnP / FUSED.

    cost = sum_i [ ||r_reproj,i||^2 / sigma_px^2 + ||r_depth,i||^2 / sigma_depth,i^2 ]

    r_reproj,i (2D, px, 2 components): projectPoints(p_base_i) - clicked
        pixel_i, using the current transform estimate -- exactly the
        solvePnP residual.
    r_depth,i (3D, m, 3 components): (R_base_cam @ p_cam_i + t_base_cam) -
        p_base_i, i.e. the clicked pixel's depth back-projected to a
        camera-frame point, transformed by the current transform estimate,
        compared against the known base-frame point -- the same full
        point-matching residual as solve_fused()'s depth term (richer than
        a bare depth/z-only check: it also constrains lateral x/y camera
        position, not just range).
    sigma_px: constant assumed pixel click/detection noise (kept constant --
        click/detection pixel precision doesn't scale much with range,
        unlike depth noise).
    sigma_depth,i: range-proportional depth noise, applied isotropically to
        all 3 components of r_depth,i (same simplifying assumption
        solve_fused() already made) --
        sigma_depth,i = clamp(depth_rel_error * z_i, min_depth_sigma_m)
        (same principle as Stage 2's R modeling: D435-style depth noise
        grows roughly linearly with range). This is what makes HYBRID_2D3D
        different from FUSED: FUSED uses one constant sigma_3d_mm for every
        pose regardless of how far the camera was from it; here every pose
        gets its own weight based on its actual measured range, so poses
        captured farther from the camera (noisier depth) are automatically
        down-weighted relative to closer ones.

    NOTE on the surface-vs-axis R/sin(theta) angle term: this project also
    has that geometry (see calib_verify_3d.py's _predicted_gap_m(), used
    for the LIVE VERIFICATION overlay on forearm_link/wrist_1_link). It is
    intentionally NOT included here. The only correspondence point this
    solver ever uses is calibration_point/tool0 (calibration_point_publisher:
    zero offset from tool0 = the flat flange face), which calib_verify_3d.py
    itself treats as R=0 -- i.e. the angle term would be architecturally
    inert for this solver's actual input regardless. Extending correspondences
    to forearm_link/wrist_1_link (where the angle term is real) would need
    per-pose orientation in manifest.json, which is out of scope (manifest
    format is frozen). Keep using calib_verify_3d.py for the angle-aware
    check against those links; this solver stays distance-only by design.
    """
    entries_d, p_cam_arr, p_base_d, _pixels_d = _backproject_entries(entries, p_base, K, depth_scale)
    if len(entries_d) < 4:
        raise RuntimeError(
            f'Need >= 4 confirmed training correspondences with usable depth for '
            f'HYBRID_2D3D solve, got {len(entries_d)} (out of {len(entries)} confirmed)')
    z_meas = [pc[2] for pc in p_cam_arr]  # camera-frame z at the clicked pixel, for the sigma model

    try:
        seed = solve_3d3d(manifest, entries, p_base, K)
        T_base_cam_seed = seed['T_base_cam']
        seed_method = 'KABSCH_3D3D'
    except RuntimeError as ex:
        print(f'HYBRID_2D3D: Kabsch seed unavailable ({ex}), falling back to PnP (ITERATIVE) seed')
        seed = solve(manifest, 'ITERATIVE')
        T_base_cam_seed = seed['T_base_cam']
        seed_method = 'ITERATIVE (PnP fallback)'
    print(f'HYBRID_2D3D: seeded from {seed_method}')

    sigma_list = [max(depth_rel_error * z, min_depth_sigma_m) for z in z_meas]

    p_cam_lookup = {id(e): pc for e, pc in zip(entries_d, p_cam_arr)}
    sigma_lookup = {id(e): s for e, s in zip(entries_d, sigma_list)}

    T_cam_base_seed = np.linalg.inv(T_base_cam_seed)
    rvec0, _ = cv2.Rodrigues(T_cam_base_seed[:3, :3])
    x0 = np.concatenate([rvec0.flatten(), T_cam_base_seed[:3, 3].flatten()])

    pixels_all = np.array([e['pixel_uv'] for e in entries], dtype=float)

    def residuals(x):
        rvec = x[:3].reshape(3, 1)
        tvec = x[3:6].reshape(3, 1)
        proj, _ = cv2.projectPoints(p_base, rvec, tvec, K, D)
        proj = proj.reshape(-1, 2)
        res_2d = (proj - pixels_all) / sigma_px

        R_cam_base, _ = cv2.Rodrigues(rvec)
        R_base_cam = R_cam_base.T
        t_base_cam = -R_base_cam @ tvec.flatten()
        res_depth = []
        for e, pb in zip(entries, p_base):
            pc = p_cam_lookup.get(id(e))
            sig = sigma_lookup.get(id(e))
            if pc is None or sig is None:
                continue
            pred = R_base_cam @ pc + t_base_cam
            res_depth.append((pred - pb) / sig)
        res_depth = np.array(res_depth) if res_depth else np.empty((0, 3))
        return np.concatenate([res_2d.flatten(), res_depth.flatten()])

    result = least_squares(residuals, x0, method='lm')
    rvec = result.x[:3].reshape(3, 1)
    tvec = result.x[3:6].reshape(3, 1)
    R_cam_base, _ = cv2.Rodrigues(rvec)
    T_cam_base = np.eye(4)
    T_cam_base[:3, :3] = R_cam_base
    T_cam_base[:3, 3] = tvec.flatten()
    T_base_cam = np.linalg.inv(T_cam_base)

    errors_2d, _ = _reprojection_errors(p_base, pixels_all, rvec, tvec, K, D)
    rms_2d = float(np.sqrt(np.mean(errors_2d ** 2)))

    R_base_cam, t_base_cam = T_base_cam[:3, :3], T_base_cam[:3, 3]
    pred_3d = (R_base_cam @ p_cam_arr.T).T + t_base_cam
    dist_mm = np.linalg.norm(pred_3d - p_base_d, axis=1) * 1000
    rms_3d_mm = float(np.sqrt(np.mean(dist_mm ** 2)))

    print(f'\n=== HYBRID_2D3D solve (uncertainty-weighted joint 2D+3D) ===')
    print(f'N correspondences: {len(entries)} total, {len(entries_d)} with usable depth')
    print(f'sigma_px={sigma_px:.2f}px  depth_rel_error={depth_rel_error * 100:.1f}%  '
          f'min_depth_sigma={min_depth_sigma_m * 1000:.1f}mm')
    print(f'RMS reprojection error @ fused solution: {rms_2d:.3f} px')
    print(f'RMS 3D (depth-backprojection) error @ fused solution: {rms_3d_mm:.2f} mm')
    print(f'optimizer: status={result.status} cost={result.cost:.4f} nfev={result.nfev}')

    print(f'\nPer-pose depth-term weighting:')
    print(f'{"pose":>5} {"z_m":>7} {"sigma_depth_mm":>14}')
    for e, z, s in zip(entries_d, z_meas, sigma_list):
        print(f'{e["pose_index"]:>5} {z:7.3f} {s * 1000:14.2f}')

    return {
        'entries': entries, 'p_base': p_base, 'pixels': pixels_all,
        'rvec': rvec, 'tvec': tvec,
        'T_cam_base': T_cam_base, 'T_base_cam': T_base_cam,
        'rms_2d_px': rms_2d, 'rms_3d_mm': rms_3d_mm,
        'seed_method': seed_method,
    }


def cross_check_3d3d(manifest, entries, p_base):
    K, _ = _camera_matrix(manifest['camera_info'])
    entries_used, p_cam_arr, p_base_arr, _ = _backproject_entries(entries, p_base, K)

    if len(entries_used) < 4:
        print(f'\n=== 3D-3D cross-check === skipped: only {len(entries_used)} usable '
              f'depth points (need >= 4)')
        return

    R, t = _kabsch_umeyama(p_cam_arr, p_base_arr)  # maps p_cam -> p_base i.e. R,t of T_base_cam
    T_base_cam_3d3d = np.eye(4)
    T_base_cam_3d3d[:3, :3] = R
    T_base_cam_3d3d[:3, 3] = t

    residuals = np.linalg.norm((R @ p_cam_arr.T).T + t - p_base_arr, axis=1)
    print(f'\n=== 3D-3D cross-check (independent estimate, N={len(entries_used)}) ===')
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


def run_validate(manifest, T_base_cam, K, D, max_err_px, max_err_mm=10.0):
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

    entries_d, p_cam_arr, p_base_arr, _ = _backproject_entries(entries, p_base, K)
    if len(entries_d) == 0:
        return
    R_base_cam, t_base_cam = T_base_cam[:3, :3], T_base_cam[:3, 3]
    p_pred = (R_base_cam @ p_cam_arr.T).T + t_base_cam
    dist_mm = np.linalg.norm(p_pred - p_base_arr, axis=1) * 1000

    print(f'\n=== Validation, depth-based 3D check (N={len(entries_d)}) ===')
    for e, d in zip(entries_d, dist_mm):
        passed = d <= max_err_mm
        print(f'  pose {e["pose_index"]:>3}  3D error={d:6.1f}mm  '
              f'{"PASS" if passed else "FAIL"}')
    rms_mm = float(np.sqrt(np.mean(dist_mm ** 2)))
    print(f'Validation RMS (3D): {rms_mm:.2f}mm (threshold {max_err_mm:.1f}mm)')
    print(f'OVERALL 3D VALIDATION RESULT: {"PASS" if rms_mm <= max_err_mm else "FAIL"}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--method', default='FUSED',
                         choices=list(PNP_METHODS.keys()) + ['KABSCH_3D3D', 'FUSED', 'HYBRID_2D3D'],
                         help='FUSED (default): joint 2D+3D nonlinear least-squares solve that '
                              'optimizes reprojection error AND depth-backprojected 3D error '
                              'together with constant weights. HYBRID_2D3D: same joint 2D+3D '
                              'idea but with a per-pose depth weight (range-proportional noise '
                              'model) instead of a constant one -- see solve_hybrid_2d3d() '
                              'docstring; opt-in only. KABSCH_3D3D: '
                              'depth-only rigid-transform solve (SVD). ITERATIVE/EPNP/SQPNP/P3P: '
                              'classic 2D-only solvePnP, ignores depth.')
    parser.add_argument('--sigma-2d-px', type=float, default=1.5,
                         help='FUSED only: assumed 1-sigma pixel measurement noise, used to '
                              'weight the 2D residual relative to the 3D one.')
    parser.add_argument('--sigma-3d-mm', type=float, default=8.0,
                         help='FUSED only: assumed 1-sigma depth measurement noise (mm) at '
                              'operating distance, used to weight the 3D residual relative to '
                              'the 2D one.')
    parser.add_argument('--hybrid-sigma-px', type=float, default=1.5,
                         help='HYBRID_2D3D only: assumed 1-sigma pixel click/detection noise.')
    parser.add_argument('--hybrid-depth-rel-error', type=float, default=0.02,
                         help='HYBRID_2D3D only: D435-style depth noise as a fraction of range '
                              '(default 2%%), the distance term of sigma_depth,i.')
    parser.add_argument('--hybrid-min-depth-sigma-m', type=float, default=0.005,
                         help='HYBRID_2D3D only: floor for sigma_depth,i (range-proportional '
                              'depth noise only -- no angle term, see solve_hybrid_2d3d() '
                              'docstring for why; use calib_verify_3d.py for the angle-aware '
                              'live verification check).')
    parser.add_argument('--max-reprojection-error-px', type=float, default=5.0)
    parser.add_argument('--max-3d-error-mm', type=float, default=10.0,
                         help='PASS/FAIL threshold for the depth-based 3D validation check '
                              '(tightened from the original 30mm hysteresis-margin default '
                              'per user request 2026-09-04).')
    parser.add_argument('--cross-check-3d3d', action='store_true',
                         help='Also print the PnP-vs-3D3D side-by-side comparison. Ignored '
                              '(redundant) when --method KABSCH_3D3D or FUSED is already used.')
    parser.add_argument('--validate', action='store_true')
    parser.add_argument('--out', default=None)
    parser.add_argument('--base-frame', default='base')
    parser.add_argument('--camera-frame', default='camera_color_optical_frame')
    args = parser.parse_args()

    manifest_path = os.path.expanduser(args.manifest)
    manifest = _load_manifest(manifest_path)
    if manifest.get('camera_info') is None:
        raise RuntimeError('manifest has no cached camera_info; run image_capture_node first')

    K, D = _camera_matrix(manifest['camera_info'])
    if args.method == 'FUSED':
        entries, p_base, _pixels = _collect_correspondences(manifest, want_validation=False)
        solved = solve_fused(manifest, entries, p_base, K, D,
                              sigma_2d_px=args.sigma_2d_px, sigma_3d_mm=args.sigma_3d_mm)
    elif args.method == 'HYBRID_2D3D':
        entries, p_base, _pixels = _collect_correspondences(manifest, want_validation=False)
        solved = solve_hybrid_2d3d(
            manifest, entries, p_base, K, D,
            sigma_px=args.hybrid_sigma_px,
            depth_rel_error=args.hybrid_depth_rel_error,
            min_depth_sigma_m=args.hybrid_min_depth_sigma_m)
    elif args.method == 'KABSCH_3D3D':
        entries, p_base, _pixels = _collect_correspondences(manifest, want_validation=False)
        solved = solve_3d3d(manifest, entries, p_base, K)
    else:
        solved = solve(manifest, args.method)

    report_reprojection(
        solved['entries'], solved['p_base'], solved['pixels'],
        solved['rvec'], solved['tvec'], K, D,
        args.max_reprojection_error_px, label='training set')

    if args.cross_check_3d3d:
        if args.method in ('KABSCH_3D3D', 'FUSED', 'HYBRID_2D3D'):
            print(f'\n(--cross-check-3d3d skipped: --method is already {args.method}, '
                  f'the cross-check would recompute a redundant depth-only estimate)')
        else:
            cross_check_3d3d(manifest, solved['entries'], solved['p_base'])

    out_path = args.out or os.path.join(os.path.dirname(manifest_path), 't_base_camera.yaml')
    write_outputs(solved['T_base_cam'], out_path, args.base_frame, args.camera_frame)

    if args.validate:
        if args.method == 'HYBRID_2D3D':
            print('\n(NOTE: HYBRID_2D3D used depth in the solve itself, so the depth-based 3D '
                  'validation check below is no longer an independent cross-check -- treat the '
                  '2D reprojection RMS on held-out poses as the primary independence check.)')
        run_validate(
            manifest, solved['T_base_cam'], K, D,
            args.max_reprojection_error_px, args.max_3d_error_mm)


if __name__ == '__main__':
    main()
