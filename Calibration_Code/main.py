"""Five-tag sheet tracker. Run `python main.py --setup` to enter dimensions.

Reuses camera.py, vision.py, control.py and motors.py from Calibration_Code.
All physical inputs are millimeters. XYZ output is meters in the camera frame.
"""
import argparse
import json
import math
from pathlib import Path
import time
import cv2
import numpy as np

if __package__:
    from .camera import open_camera, read_frame, close_camera, show_frame
    from .vision import create_detector, detect_tags, load_calibration, estimate_pose
    from .control import tracking_rates
    from .motors import connect_arduino, arm_motors, send_velocity, stop_motors, disconnect_arduino
else:
    from camera import open_camera, read_frame, close_camera, show_frame
    from vision import create_detector, detect_tags, load_calibration, estimate_pose
    from control import tracking_rates
    from motors import connect_arduino, arm_motors, send_velocity, stop_motors, disconnect_arduino

DEFAULT_CONFIG = {
    'camera': 0, 'width': 1280, 'height': 720, 'family': '36h11',
    'paper_width_mm': 215.9, 'paper_height_mm': 279.4,
    'tags': [
        {'name': 'top_left', 'id': 0, 'size_mm': 30, 'x_mm': 25, 'y_mm': 25, 'rotation_deg': 0},
        {'name': 'top_right', 'id': 1, 'size_mm': 30, 'x_mm': 190.9, 'y_mm': 25, 'rotation_deg': 0},
        {'name': 'bottom_right', 'id': 2, 'size_mm': 30, 'x_mm': 190.9, 'y_mm': 254.4, 'rotation_deg': 0},
        {'name': 'bottom_left', 'id': 3, 'size_mm': 30, 'x_mm': 25, 'y_mm': 254.4, 'rotation_deg': 0},
        {'name': 'center', 'id': 4, 'size_mm': 30, 'x_mm': 107.95, 'y_mm': 139.7, 'rotation_deg': 0},
    ],
    'calibration_file': None, 'serial_port': 'COM3',
    'pan_direction': 1, 'tilt_direction': 1,
    'min_visible_tags': 2, 'max_reprojection_error_px': 3.0,
}


def tag_sheet_corners(spec):
    """Decoded TL,TR,BR,BL corners on paper, in meters; rotation CW on page."""
    half = float(spec['size_mm']) / 2
    corners = np.array([[-half, -half], [half, -half], [half, half], [-half, half]])
    a = math.radians(spec.get('rotation_deg', 0))
    r = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    corners = corners @ r.T + [spec['x_mm'], spec['y_mm']]
    return np.column_stack((corners / 1000, np.zeros(4)))


def validate_config(config):
    """Reject duplicate IDs, invalid dimensions, overlaps and out-of-page tags."""
    for key in ('paper_width_mm', 'paper_height_mm', 'max_reprojection_error_px'):
        if not math.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f'{key} must be positive and finite')
    if len(config['tags']) != 5:
        raise ValueError('Configure exactly five tags: four corners and one center')
    ids = [t['id'] for t in config['tags']]
    if any(type(i) is not int or i < 0 for i in ids) or len(set(ids)) != 5:
        raise ValueError('All five IDs must be different nonnegative integers')
    sizes = {'16h5': 30, '25h9': 35, '36h10': 2320, '36h11': 587}
    if config['family'] not in sizes or max(ids) >= sizes[config['family']]:
        raise ValueError('Unknown family or ID outside that family dictionary')
    if type(config['min_visible_tags']) is not int or not 1 <= config['min_visible_tags'] <= 5:
        raise ValueError('min_visible_tags must be 1 through 5')
    if config['pan_direction'] not in (-1, 1) or config['tilt_direction'] not in (-1, 1):
        raise ValueError('Motor directions must be +1 or -1')
    polygons = []
    for tag in config['tags']:
        for key in ('size_mm', 'x_mm', 'y_mm', 'rotation_deg'):
            if not math.isfinite(tag[key]):
                raise ValueError(f'Invalid {key} for tag {tag["id"]}')
        if tag['size_mm'] <= 0:
            raise ValueError('Tag sizes must be positive')
        poly = tag_sheet_corners(tag)[:, :2] * 1000
        if np.any(poly < -1e-6) or np.any(poly > [config['paper_width_mm'] + 1e-6,
                                                config['paper_height_mm'] + 1e-6]):
            raise ValueError(f'Tag {tag["id"]} extends outside the paper')
        for other in polygons:
            area, _ = cv2.intersectConvexConvex(poly.astype(np.float32), other)
            if area > .001:
                raise ValueError('Tags overlap; check their center coordinates and sizes')
        polygons.append(poly.astype(np.float32))
    return config


def enter_config(path):
    """Interactive layout entry. Writes inputs only; never connects to hardware."""
    config = json.loads(json.dumps(DEFAULT_CONFIG))
    def ask(label, default, cast=float):
        text = input(f'{label} [{default}]: ').strip()
        return cast(text) if text else default
    print('Use millimeters. Tag positions are CENTERS measured from paper top-left.')
    print('Size is outer BLACK square side, excluding white margin. Rotation is clockwise.')
    config['paper_width_mm'] = ask('Paper width', 215.9)
    config['paper_height_mm'] = ask('Paper height', 279.4)
    config['family'] = ask('AprilTag family', '36h11', str)
    margin = ask('Default corner-tag CENTER distance from each paper edge', 25.0)
    w, h = config['paper_width_mm'], config['paper_height_mm']
    positions = [(margin, margin), (w-margin, margin), (w-margin, h-margin),
                 (margin, h-margin), (w/2, h/2)]
    for tag, (x, y) in zip(config['tags'], positions):
        print(f'\n{tag["name"]}')
        tag['id'] = ask('  ID', tag['id'], int)
        tag['size_mm'] = ask('  Black-square side', 30.0)
        tag['x_mm'] = ask('  Center X from left edge', x)
        tag['y_mm'] = ask('  Center Y from top edge', y)
        tag['rotation_deg'] = ask('  Clockwise rotation (0 for all upright)', 0.0)
    config['serial_port'] = ask('Arduino serial port (used only with --motors)', 'COM3', str)
    calibration = ask('Calibration JSON path (leave blank for pixel-only preview)', '', str)
    config['calibration_file'] = calibration or None
    validate_config(config)
    path.write_text(json.dumps(config, indent=2), encoding='utf-8')
    print(f'Saved {path}. Run again without --setup to open the camera.')


def collect_correspondences(tags, config):
    """Return sheet 3D points and matching detected image points."""
    specs = {t['id']: t for t in config['tags']}
    # Duplicate IDs in an image are ambiguous; do not use those observations.
    counts = {t.tag_id: sum(x.tag_id == t.tag_id for x in tags) for t in tags}
    known = [t for t in tags if t.tag_id in specs and counts[t.tag_id] == 1]
    if not known:
        return np.empty((0, 3)), np.empty((0, 2)), []
    obj = np.concatenate([tag_sheet_corners(specs[t.tag_id]) for t in known])
    img = np.concatenate([t.corners for t in known]).astype(np.float64)
    return obj, img, known


def locate_sheet(tags, config, calibration=None):
    """Project actual paper center from known layout; never average tag centers.

    With calibration returns camera-relative sheet-center XYZ and fitted rotation.
    Without calibration returns an approximate pixel center using homography.
    Each accepted marker must have all four corners consistent with the layout.
    """
    obj, img, known = collect_correspondences(tags, config)
    if len(known) < config['min_visible_tags']:
        return None
    limit = config['max_reprojection_error_px']
    plane_img = img
    if calibration:
        plane_img = cv2.undistortPoints(img.reshape(-1, 1, 2), calibration.matrix,
                                       calibration.distortion, P=calibration.matrix).reshape(-1, 2)
    homography, mask = cv2.findHomography(obj[:, :2], plane_img, cv2.RANSAC, limit)
    if homography is None or mask is None or not np.isfinite(homography).all():
        return None
    good_tags = [i for i in range(len(known)) if mask.reshape(-1, 4)[i].all()]
    if len(good_tags) < config['min_visible_tags']:
        return None
    indexes = np.concatenate([np.arange(i*4, i*4+4) for i in good_tags])
    obj, img = obj[indexes], img[indexes]
    center_obj = np.array([[config['paper_width_mm']/2000,
                            config['paper_height_mm']/2000, 0.]], np.float64)
    used_ids = [known[i].tag_id for i in good_tags]
    if not calibration:
        homography, _ = cv2.findHomography(obj[:, :2], img, 0)
        if homography is None:
            return None
        center = cv2.perspectiveTransform(center_obj[:, :2].reshape(1, 1, 2), homography).reshape(2)
        if not np.isfinite(center).all():
            return None
        return {'center_px': center, 'xyz_m': None, 'rvec': None, 'used_ids': used_ids}
    fits = cv2.solvePnPGeneric(obj, img, calibration.matrix, calibration.distortion,
                               flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    if fits[0]:
        for rvec, tvec in zip(fits[1], fits[2]):
            rotation, _ = cv2.Rodrigues(rvec)
            xyz = (rotation @ center_obj[0]) + tvec.ravel()
            if np.any((obj @ rotation.T + tvec.reshape(1, 3))[:, 2] <= 0) or xyz[2] <= 0:
                continue
            projection, _ = cv2.projectPoints(obj, rvec, tvec, calibration.matrix, calibration.distortion)
            error = np.linalg.norm(projection.reshape(-1, 2)-img, axis=1)
            if not np.isfinite(error).all() or np.max(error) > limit:
                continue
            center, _ = cv2.projectPoints(center_obj, rvec, tvec, calibration.matrix, calibration.distortion)
            candidates.append((float(np.mean(error)), {'center_px': center.reshape(2),
                'xyz_m': xyz, 'rvec': rvec, 'used_ids': used_ids}))
    return min(candidates, key=lambda x: x[0])[1] if candidates else None


def annotate_sheet(frame, tags, config, sheet, calibration, status):
    """Draw detections plus a separate panel showing all five tag positions."""
    out = frame.copy()
    h, w = out.shape[:2]
    cv2.drawMarker(out, (w//2, h//2), (255,255,0), cv2.MARKER_CROSS, 25, 2)
    used = set(sheet['used_ids']) if sheet else set()
    for tag in tags:
        color = (0,255,0) if tag.tag_id in used else (0,170,255)
        cv2.polylines(out, [np.rint(tag.corners).astype(np.int32)], True, color, 2)
        xy = tuple(np.rint(tag.center).astype(int))
        cv2.putText(out, f'ID {tag.tag_id}', xy, cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
    if sheet:
        xy = tuple(np.rint(sheet['center_px']).astype(int))
        # Avoid drawing invalid gigantic coordinates when projection is far offscreen.
        if -w <= xy[0] <= 2*w and -h <= xy[1] <= 2*h:
            cv2.circle(out, xy, 12, (255,0,255), 2)
            cv2.line(out, (w//2,h//2), xy, (255,0,255), 2)
    canvas = np.zeros((max(h, 620), w+510, 3), np.uint8)
    canvas[:h, :w] = out
    lines = ['FIVE-TAG PAPER TRACKER', status, 'A: track/pause | Space: stop | Q: quit',
             f'Paper: {config["paper_width_mm"]} x {config["paper_height_mm"]} mm',
             'Aim: geometric center of paper', '']
    if sheet:
        ex, ey = sheet['center_px'] - [w/2, h/2]
        lines += [f'Center error: {ex:+.1f}, {ey:+.1f} px', f'Layout-fit IDs: {sorted(used)}']
        if sheet['xyz_m'] is not None:
            x,y,z = sheet['xyz_m']
            lines += [f'Paper XYZ m: {x:+.3f}, {y:+.3f}, {z:.3f}']
        else:
            lines += ['Paper XYZ: calibration required', 'Pixel-only fit: approximate']
    else:
        lines += ['Paper center: no valid layout fit', 'Motors stopped when tracking target lost']
    lines += ['', 'Each tag: camera-relative XYZ in meters']
    for spec in config['tags']:
        matches = [t for t in tags if t.tag_id == spec['id']]
        label = f'{spec["name"]} (ID {spec["id"]})'
        lines.append(label)
        if len(matches) != 1:
            lines.append('  Not visible / duplicate ID')
        elif calibration:
            pose = estimate_pose(matches[0].corners, spec['size_mm']/1000, calibration)
            if pose:
                x,y,z = pose.tvec.ravel()
                lines.append(f'  {x:+.3f}, {y:+.3f}, {z:.3f}')
            else:
                lines.append('  Pose rejected')
        else:
            x,y = matches[0].center
            lines.append(f'  Pixel center {x:.0f}, {y:.0f}; XYZ unavailable')
    for i, line in enumerate(lines):
        cv2.putText(canvas, line, (w+12, 25+i*23), cv2.FONT_HERSHEY_SIMPLEX, .48, (255,255,255), 1)
    return canvas


def run(config, calibration=None, motors_enabled=False):
    """Integration loop. Preview by default; hardware is passed/configured explicitly."""
    camera = connection = None
    tracking = False
    last_command = 0.
    status = 'Preview: motors not armed'
    detector = create_detector(config['family'])
    try:
        camera = open_camera(config['camera'], config['width'], config['height'])
        if motors_enabled:
            connection = connect_arduino(config['serial_port'])
        while True:
            frame = read_frame(camera)
            h,w = frame.shape[:2]
            if calibration and (w,h) != (calibration.width,calibration.height):
                raise ValueError('Camera resolution differs from calibration')
            tags = detect_tags(frame, detector)
            sheet = locate_sheet(tags, config, calibration)
            # Never drive toward an inferred center outside the actual image.
            valid = sheet is not None and 0 <= sheet['center_px'][0] < w and 0 <= sheet['center_px'][1] < h
            if tracking and not valid:
                stop_motors(connection)
                tracking = False
                status = 'Target lost: stopped. Reacquire then A.'
            now = time.monotonic()
            if tracking and now-last_command >= .05:
                rates = tracking_rates(sheet['center_px'], (w,h), config['pan_direction'],config['tilt_direction'])
                send_velocity(connection,*rates)
                last_command = now
                status = f'Tracking | pan {rates[0]:+.2f}, tilt {rates[1]:+.2f} deg/s'
            key = show_frame(annotate_sheet(frame,tags,config,sheet,calibration,status), 'Paper tracker')
            if key == ord('q') or cv2.getWindowProperty('Paper tracker',cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord(' ') or (key == ord('a') and tracking):
                if connection:
                    stop_motors(connection)
                tracking = False
                status = 'Stopped; holding position'
            elif key == ord('a'):
                if connection and valid and calibration is not None:
                    arm_motors(connection)
                    tracking = True
                    last_command = 0.
                else:
                    status = 'Need motor option, calibration and valid visible layout'
    finally:
        try:
            disconnect_arduino(connection)
        finally:
            close_camera(camera)
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('sheet_config.json'))
    parser.add_argument('--setup', action='store_true', help='Enter all five tag dimensions interactively')
    parser.add_argument('--motors', action='store_true', help='Allow explicit arming using A')
    parser.add_argument('--zero-confirmed', action='store_true', help='Mount centered BEFORE Arduino startup/reset')
    args = parser.parse_args()
    if args.setup:
        enter_config(args.config)
        return
    if not args.config.exists():
        parser.error('No configuration file. Run python main.py --setup first.')
    config = validate_config(json.loads(args.config.read_text(encoding='utf-8')))
    calibration = None
    if config.get('calibration_file'):
        path = Path(config['calibration_file'])
        calibration = load_calibration(path if path.is_absolute() else args.config.parent/path)
    if args.motors and (not args.zero_confirmed or calibration is None):
        parser.error('--motors requires --zero-confirmed and calibration_file in config')
    run(config, calibration, args.motors)


if __name__ == '__main__':
    main()
