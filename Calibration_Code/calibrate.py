"""Estimate webcam intrinsics from diverse chessboard photos, no Arduino needed."""
import argparse
import glob
import json
import cv2
import numpy as np


def calibrate_images(paths, columns, rows, square_m):
    if columns < 2 or rows < 2 or square_m <= 0:
        raise ValueError('Use >=2 inner corners each axis and positive square size')
    obj = np.zeros((columns * rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_m
    objects, points = [], []
    size = None
    for path in paths:
        image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f'Cannot read {path}')
        current = image.shape[::-1]
        if size is not None and current != size:
            raise ValueError('All calibration images must have identical resolution')
        size = current
        ok, corners = cv2.findChessboardCornersSB(image, (columns, rows))
        if ok:
            objects.append(obj.copy()); points.append(corners)
    if len(points) < 10:
        raise ValueError(f'Only {len(points)} usable boards; need >=10 diverse views')
    rms, k, d, _, _ = cv2.calibrateCamera(objects, points, size, None, None)
    return dict(camera_matrix=k.tolist(), dist_coeffs=d.ravel().tolist(),
                image_size=list(size), rms_px=float(rms), usable_views=len(points))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--images', required=True, help='Glob: calibration_images/*.png')
    p.add_argument('--columns', type=int, required=True, help='INNER corner count')
    p.add_argument('--rows', type=int, required=True, help='INNER corner count')
    p.add_argument('--square-m', type=float, required=True)
    p.add_argument('--output', default='calibration.json')
    a = p.parse_args()
    result = calibrate_images(sorted(glob.glob(a.images)), a.columns, a.rows, a.square_m)
    with open(a.output, 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2)
    print(f"Saved {a.output}; RMS={result['rms_px']:.3f} px. Inspect calibration quality before use.")
