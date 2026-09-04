#!/usr/bin/env python3
"""Jetson script: automatic calibration-point pixel detection.

Loads each image/depth pair referenced in a manifest.json (written by
image_capture_node) and automatically finds the pixel location of the
physical calibration point (EE flange) using depth, no manual clicking.

How it works (no manual click, no color/shape detection):
  1. Keep a running estimate of T_base<-camera, starting from a rough seed
     (position guessed from the robot's depth-cluster centroid, rotation
     reused from the last known calibration -- see conversation/context).
  2. For each pose, project its known p_base (from FK) into a predicted
     pixel using the running estimate.
  3. Search a small window around that predicted pixel in the depth image
     for pixels whose depth is close to the predicted depth (a coherent
     surface, not noise/background) -- their centroid is the detected
     pixel. This directly reuses the same depth back-projection style as
     solve_calibration.py / calib_verify_3d.py.
  4. If found, confirm it and back-project it to a camera-frame 3D point.
     Every few confirmed points, re-solve the running T_base<-camera via
     Kabsch/Umeyama (same as solve_calibration.py's KABSCH_3D3D) using all
     confirmed points so far -- later poses get searched with a
     progressively more accurate estimate than the rough seed.
  5. If a pose's window has no coherent depth cluster (occluded, out of
     frame, background clutter), it is skipped (click_status='skipped',
     pixel_uv=None) and the sequence continues -- no manual fallback.

Usage:
    ee_click_tool --manifest ~/calib_ws/calib_data/<run>/manifest.json \
        --seed-translation X Y Z --seed-quat X Y Z W
"""
import argparse
import json
import os

import cv2
import numpy as np

DEPTH_SCALE = 0.001  # 16UC1 mm -> m
# Narrow defaults: seed now comes from 6 manually-clicked, depth-verified
# points (1.75mm RMS 3D residual) instead of a blind geometric guess, so a
# tight window is appropriate and avoids latching onto neighboring links.
SEARCH_RADIUS_PX = 25
DEPTH_TOL_M = 0.03
MIN_CLUSTER_PX = 15
REESTIMATE_EVERY = 3
MIN_POINTS_TO_REESTIMATE = 6


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _kabsch_umeyama(src_pts, dst_pts):
    """Rigid (no-scale) transform mapping src_pts -> dst_pts via SVD.
    Returns (R, t) such that dst ~= R @ src + t. Same as solve_calibration.py.
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


def _camera_matrix(camera_info):
    K = np.array(camera_info['k'], dtype=float).reshape(3, 3)
    return K


def detect_ee_pixel_depth(depth_img, K, p_base, T_base_cam,
                           search_radius_px=SEARCH_RADIUS_PX,
                           depth_tol_m=DEPTH_TOL_M, min_cluster_px=MIN_CLUSTER_PX):
    """Project p_base into the image via the current T_base_cam estimate,
    then look for a depth-coherent cluster near that pixel. Returns
    (u, v, p_cam) on success, None if nothing usable was found.
    """
    R_base_cam = T_base_cam[:3, :3]
    t_base_cam = T_base_cam[:3, 3]
    R_cam_base = R_base_cam.T
    t_cam_base = -R_cam_base @ t_base_cam

    p_cam_pred = R_cam_base @ p_base + t_cam_base
    if p_cam_pred[2] <= 0.05:
        return None
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    u_pred = fx * p_cam_pred[0] / p_cam_pred[2] + cx
    v_pred = fy * p_cam_pred[1] / p_cam_pred[2] + cy
    expected_depth_m = p_cam_pred[2]

    h, w = depth_img.shape
    u0 = max(0, int(u_pred - search_radius_px))
    u1 = min(w, int(u_pred + search_radius_px))
    v0 = max(0, int(v_pred - search_radius_px))
    v1 = min(h, int(v_pred + search_radius_px))
    if u1 <= u0 or v1 <= v0:
        return None

    window = depth_img[v0:v1, u0:u1].astype(float) * DEPTH_SCALE
    mask = (window > 0) & (np.abs(window - expected_depth_m) < depth_tol_m)
    if mask.sum() < min_cluster_px:
        return None

    ys, xs = np.where(mask)
    u_det = float(xs.mean()) + u0
    v_det = float(ys.mean()) + v0
    z_det = float(window[ys, xs].mean())
    x_cam = (u_det - cx) * z_det / fx
    y_cam = (v_det - cy) * z_det / fy
    p_cam = np.array([x_cam, y_cam, z_det])
    return u_det, v_det, p_cam


def _load_manifest(path):
    with open(path) as f:
        return json.load(f)


def _save_manifest(path, manifest):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(manifest, f, indent=2)
    os.replace(tmp, path)


def run_auto(manifest_path, seed_translation, seed_quat):
    manifest = _load_manifest(manifest_path)
    entries = manifest['entries']
    K = _camera_matrix(manifest['camera_info'])

    T_base_cam = np.eye(4)
    T_base_cam[:3, :3] = quat_to_R(*seed_quat)
    T_base_cam[:3, 3] = seed_translation
    print(f'Seed T_base<-camera translation={seed_translation} quat={seed_quat}')

    confirmed_p_cam, confirmed_p_base = [], []
    n_confirmed = n_skipped = 0

    for entry in entries:
        if not entry.get('success'):
            continue
        if entry.get('click_status') in ('confirmed', 'auto_confirmed'):
            # keep pre-existing confirmed points in the re-estimation pool by
            # re-back-projecting their stored pixel through their depth frame
            if entry.get('pixel_uv') is not None and entry.get('depth_path') and os.path.exists(entry['depth_path']):
                depth_img = cv2.imread(entry['depth_path'], cv2.IMREAD_UNCHANGED)
                if depth_img is not None:
                    u, v = entry['pixel_uv']
                    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
                    z = float(depth_img[int(round(v)), int(round(u))]) * DEPTH_SCALE
                    if z > 0:
                        p_cam = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
                        pb = np.array([entry['p_base']['x'], entry['p_base']['y'], entry['p_base']['z']])
                        confirmed_p_cam.append(p_cam)
                        confirmed_p_base.append(pb)
            continue

        depth_img = cv2.imread(entry['depth_path'], cv2.IMREAD_UNCHANGED)
        if depth_img is None:
            print(f'[pose {entry["pose_index"]}] could not read depth {entry["depth_path"]}, skipping')
            entry['click_status'] = 'skipped'
            entry['pixel_uv'] = None
            n_skipped += 1
            continue

        p_base = np.array([entry['p_base']['x'], entry['p_base']['y'], entry['p_base']['z']])
        det = detect_ee_pixel_depth(depth_img, K, p_base, T_base_cam)

        if det is None:
            print(f'[pose {entry["pose_index"]}] auto-detect FAILED, skipping')
            entry['click_status'] = 'skipped'
            entry['pixel_uv'] = None
            n_skipped += 1
            continue

        u, v, p_cam = det
        entry['pixel_uv'] = [u, v]
        entry['click_status'] = 'confirmed'
        print(f'[pose {entry["pose_index"]}] auto-detected pixel=({u:.1f},{v:.1f})')
        n_confirmed += 1
        confirmed_p_cam.append(p_cam)
        confirmed_p_base.append(p_base)

        if n_confirmed >= MIN_POINTS_TO_REESTIMATE and n_confirmed % REESTIMATE_EVERY == 0:
            R, t = _kabsch_umeyama(np.array(confirmed_p_cam), np.array(confirmed_p_base))
            T_base_cam = np.eye(4)
            T_base_cam[:3, :3] = R
            T_base_cam[:3, 3] = t
            print(f'  -> re-estimated T_base<-camera using {n_confirmed} points, '
                  f'translation={t}')

        _save_manifest(manifest_path, manifest)

    print(f'\nDone. {n_confirmed} auto-confirmed, {n_skipped} skipped. Manifest updated: {manifest_path}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', required=True, help='Path to manifest.json')
    parser.add_argument('--seed-translation', type=float, nargs=3, required=True,
                         metavar=('X', 'Y', 'Z'))
    parser.add_argument('--seed-quat', type=float, nargs=4, required=True,
                         metavar=('X', 'Y', 'Z', 'W'))
    args = parser.parse_args()
    run_auto(os.path.expanduser(args.manifest), args.seed_translation, args.seed_quat)


if __name__ == '__main__':
    main()
