# Main entry point for a five-tag paper layout

Put main.py in Calibration_Code beside camera.py, vision.py, control.py and
motors.py. This package includes them, so you can also extract and run it alone.
The original app.py remains a single-tag example; main.py is the new five-tag app.
No GitHub commit is made by creating or running these files.

## Enter your paper and tag specifics

Open a terminal in Calibration_Code:

```powershell
py -m pip install -r requirements.txt
py main.py --setup
py main.py
```

Setup asks for paper width/height and, for EACH of the FIVE tags:

* ID: unique printed AprilTag ID, from the selected family.
* Size: side length of the outer black square, in millimeters. Exclude white margin.
* X: distance from the LEFT paper edge to the tag CENTER, in millimeters.
* Y: distance from the TOP paper edge to the tag CENTER, in millimeters.
* Rotation: clockwise on the paper relative to the tag's normal upright orientation.
  Use 0 when all five tags are printed upright, facing the same direction.

The four corners have independent size and position inputs; they need not have
equal dimensions. The middle tag also has independent inputs. Setup saves
sheet_config.json. Edit this JSON later or rerun --setup to change values.
Default camera 0 is the computer webcam. Edit camera/width/height in the JSON
for another device or resolution. serial_port defaults to COM3; change as needed.

Example default layout: US Letter, portrait, 215.9 x 279.4 mm, 30 mm tags:

| Position | ID | Center X mm | Center Y mm |
|---|---:|---:|---:|
| Top left | 0 | 25.00 | 25.00 |
| Top right | 1 | 190.90 | 25.00 |
| Bottom right | 2 | 190.90 | 254.40 |
| Bottom left | 3 | 25.00 | 254.40 |
| Middle | 4 | 107.95 | 139.70 |

These are EXAMPLES, not your measured dimensions. Keep white quiet margins
around each tag. Mount the paper flat. Layout fitting assumes all tags are
coplanar and the dimensions/rotations match the physical print.

## What the main file does

1. Opens the configured camera.
2. Detects tags and outlines them with IDs in the live image.
3. Fits their known paper coordinates to image corners and marks the geometric
   paper center in magenta. It does not average the observed tag centers, which
   would move the target when some tags disappear.
4. Displays each visible tag's pixel center, or calibrated camera-relative XYZ
   in meters. Also displays the paper center's camera-relative XYZ.
5. If enabled and armed, sends pan/tilt velocities to center the paper in the feed.

By default at least TWO consistent known tags must be visible to fit the paper.
The middle tag can disappear and the corner tags still locate the true center.
Unknown IDs are drawn but ignored. Duplicate IDs are excluded. Inconsistent
marker corners are rejected. If too few consistent tags remain, tracking stops.
Individual per-tag pose may remain visible even when the board fit is rejected.

## Calibration and XYZ

Run the existing calibrate.py with chessboard photos (see README.md), then set
calibration_file to its JSON path in sheet_config.json. Relative paths resolve
beside the config. Camera resolution must match calibration. A five-tag page
with entered dimensions is NOT itself a camera intrinsic calibration.

Coordinates: X right, Y down, Z forward from the camera, in METERS. They change
as the camera rotates. No world-frame coordinates or absolute motor feedback
are implied. Monocular planar pose has possible ambiguity and noise.

Without calibration you still get detection and an approximate paper-center
pixel overlay, but metric XYZ and motor arming are unavailable. Raw-image
homography does not remove lens distortion, so use calibrated mode for control.

## Run with the two motors

Upload the included arduino_pan_tilt sketch; it uses the same protocol as the
earlier helper package. Follow README.md for pins, current limits, microstepping,
gear ratios, manually centered startup zero, and direction/limit verification.

```powershell
py main.py --motors --zero-confirmed
```

Press A to arm tracking with a valid calibrated layout visible. A pauses;
Space stops and holds; Q quits. Motion starts paused. Target loss disarms and
requires A after reacquiring. Out-of-image inferred centers are not pursued.
The Arduino watchdog stops motion if commands stop arriving. After a PC stall,
Space then A re-arms. Software zero/limits do not replace physical homing.

min_visible_tags and max_reprojection_error_px are configurable in JSON. The
pan_direction and tilt_direction values are +1 or -1, determined on your mount.
Frame-center error drives control, independent of metric distance Z. Pan/tilt
aims the camera but cannot change its distance from the paper.

## Test status

Run `py -m unittest -v` from this folder. Synthetic tests exercise paper-center
recovery under perspective, center-tag occlusion, rotated tags, missing and
duplicate detections, dimensions, original tag pose, control math and serial
formatting. Camera, printed target, Arduino and motor behavior require hardware
testing. No Arduino hardware compilation or physical trial was performed here.
