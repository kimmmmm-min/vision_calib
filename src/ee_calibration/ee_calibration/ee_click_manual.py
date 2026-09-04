#!/usr/bin/env python3
"""Jetson script: manual calibration-point pixel annotation tool (limited count).

Same click UI as the original ee_click_tool.py, but capped to only process
--max-clicks entries (default 6) so a few precise manual points can be
collected to seed the automatic detector (ee_click_tool.py) without having
to click all poses by hand.

Usage:
    ee_click_manual --manifest ~/calib_ws/calib_data/<run>/manifest.json --max-clicks 6

Keys while an image is shown:
    left-click   register a candidate point, shows a zoomed preview
    c            confirm the last candidate and move to the next image
    r            redo (clear the candidate, click again)
    s            skip this pose (leaves pixel_uv=None, click_status='skipped')
    q            save and quit early
"""
import argparse
import json
import os
from typing import Optional, Tuple

import cv2
import numpy as np

ZOOM_SIZE = 160
ZOOM_SCALE = 4
WINDOW_MAIN = 'ee_click_manual (click the flange/calibration point)'
WINDOW_ZOOM = 'zoom preview'


class ClickState:
    def __init__(self):
        self.point: Optional[Tuple[int, int]] = None

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.point = (x, y)


def _zoom_crop(image: np.ndarray, center: Tuple[int, int]) -> np.ndarray:
    h, w = image.shape[:2]
    cx, cy = center
    x0 = max(0, cx - ZOOM_SIZE)
    y0 = max(0, cy - ZOOM_SIZE)
    x1 = min(w, cx + ZOOM_SIZE)
    y1 = min(h, cy + ZOOM_SIZE)
    crop = image[y0:y1, x0:x1].copy()
    if crop.size == 0:
        return np.zeros((ZOOM_SIZE * 2, ZOOM_SIZE * 2, 3), dtype=np.uint8)
    zoomed = cv2.resize(crop, None, fx=ZOOM_SCALE, fy=ZOOM_SCALE, interpolation=cv2.INTER_NEAREST)
    zh, zw = zoomed.shape[:2]
    cv2.drawMarker(zoomed, (zw // 2, zh // 2), (0, 0, 255),
                    markerType=cv2.MARKER_CROSS, markerSize=20, thickness=1)
    return zoomed


def _load_manifest(path):
    with open(path) as f:
        return json.load(f)


def _save_manifest(path, manifest):
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(manifest, f, indent=2)
    os.replace(tmp, path)


def run(manifest_path: str, max_clicks: int):
    manifest = _load_manifest(manifest_path)
    entries = manifest['entries']

    cv2.namedWindow(WINDOW_MAIN, cv2.WINDOW_NORMAL)
    state = ClickState()
    cv2.setMouseCallback(WINDOW_MAIN, state.on_mouse)

    n_done = 0
    for entry in entries:
        if n_done >= max_clicks:
            break
        if not entry.get('success'):
            continue
        if entry.get('click_status') == 'confirmed':
            continue
        image_path = entry['image_path']
        image = cv2.imread(image_path)
        if image is None:
            print(f'[pose {entry["pose_index"]}] could not read {image_path}, skipping')
            entry['click_status'] = 'skipped'
            continue

        state.point = None
        while True:
            display = image.copy()
            if state.point is not None:
                cv2.drawMarker(display, state.point, (0, 255, 0),
                                markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)
                cv2.imshow(WINDOW_ZOOM, _zoom_crop(image, state.point))
            cv2.putText(
                display,
                f'pose {entry["pose_index"]} ({n_done + 1}/{max_clicks})  '
                f'click flange center, confirm[c] redo[r] skip[s] quit[q]',
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow(WINDOW_MAIN, display)
            key = cv2.waitKey(30) & 0xFF

            if key == ord('c') and state.point is not None:
                entry['pixel_uv'] = list(state.point)
                entry['click_status'] = 'confirmed'
                _save_manifest(manifest_path, manifest)
                n_done += 1
                break
            elif key == ord('r'):
                state.point = None
            elif key == ord('s'):
                entry['pixel_uv'] = None
                entry['click_status'] = 'skipped'
                _save_manifest(manifest_path, manifest)
                n_done += 1
                break
            elif key == ord('q'):
                _save_manifest(manifest_path, manifest)
                cv2.destroyAllWindows()
                print('Saved and exiting early.')
                return

    _save_manifest(manifest_path, manifest)
    cv2.destroyAllWindows()
    print(f'Done. {n_done} poses processed. Manifest updated: {manifest_path}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', required=True, help='Path to manifest.json')
    parser.add_argument('--max-clicks', type=int, default=6)
    args = parser.parse_args()
    run(os.path.expanduser(args.manifest), args.max_clicks)


if __name__ == '__main__':
    main()
