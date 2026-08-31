#!/usr/bin/env python3
"""Jetson script: manual calibration-point pixel annotation tool.

Loads each image referenced in a manifest.json (written by image_capture_node),
lets the user click the configured physical calibration point with a magnified preview for
sub-pixel precision, and writes (u, v) back into the manifest next to its
p_base entry.

Usage:
    ee_click_tool --manifest ~/calib_ws/calib_data/<run>/manifest.json

Keys while an image is shown:
    left-click   register a candidate point, shows a zoomed preview
    c            confirm the last candidate and move to the next image
    r            redo (clear the candidate, click again)
    s            skip this pose (leaves pixel_uv=None, click_status='skipped')
    q            save and quit early

Extension point: detect_ee_pixel(image) -> Optional[(u, v)] is where a future
automatic (e.g. color-sticker) detector could plug in without touching the
rest of this tool. Not implemented here -- always returns None, so manual
clicking is the only path today.
"""
import argparse
import json
import os
from typing import Optional, Tuple

import cv2
import numpy as np

ZOOM_SIZE = 160          # half-width of the source crop shown zoomed
ZOOM_SCALE = 4
WINDOW_MAIN = 'ee_click_tool (click calibration point)'
WINDOW_ZOOM = 'zoom preview'


def detect_ee_pixel(image: np.ndarray) -> Optional[Tuple[int, int]]:
    """Extension point for a future automatic EE-pixel detector.

    Not implemented -- always returns None so ee_click_tool always falls back
    to manual clicking.
    """
    return None


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
    zoomed = cv2.resize(
        crop, None, fx=ZOOM_SCALE, fy=ZOOM_SCALE, interpolation=cv2.INTER_NEAREST)
    zh, zw = zoomed.shape[:2]
    cv2.drawMarker(
        zoomed, (zw // 2, zh // 2), (0, 0, 255),
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


def run(manifest_path: str):
    manifest = _load_manifest(manifest_path)
    entries = manifest['entries']

    cv2.namedWindow(WINDOW_MAIN, cv2.WINDOW_NORMAL)
    state = ClickState()
    cv2.setMouseCallback(WINDOW_MAIN, state.on_mouse)

    for entry in entries:
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

        auto = detect_ee_pixel(image)
        state.point = auto

        while True:
            display = image.copy()
            if state.point is not None:
                cv2.drawMarker(
                    display, state.point, (0, 255, 0),
                    markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)
                cv2.imshow(WINDOW_ZOOM, _zoom_crop(image, state.point))
            cv2.putText(
                display, f'pose {entry["pose_index"]}  click=confirm[c] redo[r] skip[s] quit[q]',
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow(WINDOW_MAIN, display)
            key = cv2.waitKey(30) & 0xFF

            if key == ord('c') and state.point is not None:
                entry['pixel_uv'] = list(state.point)
                entry['click_status'] = 'confirmed'
                _save_manifest(manifest_path, manifest)
                break
            elif key == ord('r'):
                state.point = None
            elif key == ord('s'):
                entry['pixel_uv'] = None
                entry['click_status'] = 'skipped'
                _save_manifest(manifest_path, manifest)
                break
            elif key == ord('q'):
                _save_manifest(manifest_path, manifest)
                cv2.destroyAllWindows()
                print('Saved and exiting early.')
                return

    _save_manifest(manifest_path, manifest)
    cv2.destroyAllWindows()
    print(f'Done. Manifest updated: {manifest_path}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', required=True, help='Path to manifest.json')
    args = parser.parse_args()
    run(os.path.expanduser(args.manifest))


if __name__ == '__main__':
    main()
