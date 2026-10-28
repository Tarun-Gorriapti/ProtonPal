"""AprilTag and metric-pose helpers; independent of camera/serial helpers."""
from dataclasses import dataclass
import json
import cv2
import numpy as np


@dataclass(frozen=True)
class Tag:
    tag_id: int
    corners: np.ndarray  # 4x2, decoded tag corner order
    center: np.ndarray   # 2-vector in pixels


@dataclass(frozen=True)
class Calibration:
    matrix: np.ndarray
    distortion: np.ndarray
    width: int
    height: int


@dataclass(frozen=True)
class Pose:
    rvec: np.ndarray
    tvec: np.ndarray  # x right, y down, z forward; meters
    reprojection_error_px: float


def create_detector(family="36h11"):
    families = {"16h5": cv2.aruco.DICT_APRILTAG_16h5,
                "25h9": cv2.aruco.DICT_APRILTAG_25h9,
                "36h10": cv2.aruco.DICT_APRILTAG_36h10,
                "36h11": cv2.aruco.DICT_APRILTAG_36h11}
    if family not in families:
        raise ValueError(f"Choose tag family from {tuple(families)}")
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(families[family]), params)


def detect_tags(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return []
    return [Tag(int(i), c.reshape(4, 2).copy(), c.reshape(4, 2).mean(axis=0))
            for c, i in zip(corners, ids.flatten())]


def select_tag(tags, tag_id):
    """Select only the requested ID; never silently switch tracked targets."""
    return next((tag for tag in tags if tag.tag_id == tag_id), None)


def load_calibration(path):
    data = json.loads(open(path, encoding="utf-8").read())
    k = np.asarray(data["camera_matrix"], dtype=np.float64)
    d = np.asarray(data["dist_coeffs"], dtype=np.float64).reshape(-1, 1)
    w, h = data["image_size"]
    if (k.shape != (3, 3) or not np.isfinite(k).all() or
            not np.isfinite(d).all() or d.size not in (4, 5, 8, 12, 14) or
            k[0, 0] <= 0 or k[1, 1] <= 0 or w <= 0 or h <= 0):
        raise ValueError("Invalid camera calibration")
    return Calibration(k, d, int(w), int(h))


def estimate_pose(corners, tag_size_m, calibration, max_error_px=3.0):
    """Estimate tag center relative to camera. Reject behind-camera/bad fits.

    tag_size_m is the outer BLACK square edge, excluding white margin.
    Square planar pose can be ambiguous; this is a monocular estimate.
    """
    if not np.isfinite(tag_size_m) or tag_size_m <= 0:
        raise ValueError("Tag size must be positive meters")
    s = tag_size_m / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)
    img = np.asarray(corners, np.float64).reshape(4, 2)
    result = cv2.solvePnPGeneric(obj, img, calibration.matrix,
                                calibration.distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates = []
    if result[0]:
        for r, t in zip(result[1], result[2]):
            rot, _ = cv2.Rodrigues(r)
            if not np.isfinite(t).all() or np.any((obj @ rot.T + t.reshape(1, 3))[:, 2] <= 0):
                continue
            projected, _ = cv2.projectPoints(obj, r, t, calibration.matrix, calibration.distortion)
            error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - img)**2, axis=1))))
            if np.isfinite(error) and error <= max_error_px:
                candidates.append(Pose(r, t, error))
    return min(candidates, key=lambda p: p.reprojection_error_px) if candidates else None


def draw_feedback(frame, tags, target_id, pose=None, calibration=None, message="Preview"):
    """Return annotated copy. Original image is never modified."""
    out = frame.copy()
    h, w = frame.shape[:2]
    center = (w // 2, h // 2)
    cv2.drawMarker(out, center, (255, 255, 0), cv2.MARKER_CROSS, 24, 2)
    for tag in tags:
        color = (0, 255, 0) if tag.tag_id == target_id else (0, 180, 255)
        cv2.polylines(out, [np.rint(tag.corners).astype(np.int32)], True, color, 2)
        c = tuple(np.rint(tag.center).astype(int))
        cv2.circle(out, c, 4, color, -1)
        cv2.putText(out, f"ID {tag.tag_id}", c, cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
        if tag.tag_id == target_id:
            cv2.line(out, center, c, color, 1)
            ex, ey = tag.center - np.array([w / 2, h / 2])
            cv2.putText(out, f"Offset: x={ex:+.0f}, y={ey:+.0f} px", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
    label = "XYZ unavailable: provide calibration + tag size"
    if pose is not None:
        x, y, z = pose.tvec.ravel()
        label = f"Camera-relative m: x={x:+.3f} y={y:+.3f} z={z:.3f}"
        if calibration:
            cv2.drawFrameAxes(out, calibration.matrix, calibration.distortion,
                              pose.rvec, pose.tvec, min(.05, z * .1))
    cv2.putText(out, message, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 2)
    cv2.putText(out, label, (10, h - 20), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 2)
    return out
