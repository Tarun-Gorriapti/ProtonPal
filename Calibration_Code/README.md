# AprilTag camera and Arduino pan/tilt helpers

Python helpers for webcam access, live AprilTag feedback, camera-relative XYZ,
and centering a selected tag using two STEP/DIR stepper drivers.
Python 3.10+ recommended. Default camera: computer webcam index 0.
Default tag: AprilTag 36h11 ID 0. Change both to match your actual printed tag.

## Independent modules

| File | Helpers | Hardware needed |
|---|---|---|
| camera.py | open_camera, read_frame, show_frame, close_camera | Camera |
| vision.py | create_detector, detect_tags, select_tag, load_calibration, estimate_pose, draw_feedback | None for saved images |
| control.py | center_error, axis_rate, tracking_rates | None |
| motors.py | connect_arduino, arm_motors, send_velocity, stop_motors, disconnect_arduino | Arduino for real use |
| app.py | Example wiring these independent modules together | Camera; Arduino optional |
| calibrate.py | calibrate_images | Saved chessboard images |

No helper opens a camera or serial port on import. Camera/vision modules do not
import motor code. Control math has no third-party dependencies. Instances such
as detector/camera/serial connections are passed explicitly, never global.

## 1. Install and run camera preview

Extract the ZIP and open a terminal in this directory. On Windows:

```powershell
py -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python app.py
```

Use a GUI-capable OpenCV package (not headless). In an existing environment,
avoid having multiple OpenCV distributions installed together.

```powershell
.venv\Scripts\python app.py --camera 1 --family 36h11 --tag-id 7
```

Overlay: detected IDs, tag outlines/centers, image center, pixel offsets,
and tracking status. Default capture request is 1280x720; cameras may negotiate
a different size. Q quits, Space stops, A starts/pauses tracking when configured.
Preview mode sends NO motor commands and needs NO Arduino.

## 2. Obtain camera-relative X, Y and Z

Pixel offsets work immediately. Metric XYZ requires a calibrated camera and the
physical outer black square side of your printed tag, excluding white margin.
Coordinates describe the tag center in the CURRENT camera frame:

* X positive right, Y positive down, Z positive forward, all in meters.
* Rotating the camera changes this coordinate frame. These are not room/world
  coordinates, mount encoder readings, or tag-to-screen coordinates.
* Single square pose can be ambiguous at some angles; code picks the positive
  depth solution with smallest reprojection error. Do not treat this as ground truth.

Save at least 10 (preferably 15-25) sharp photos of a measured flat chessboard
at diverse tilts/distances/positions, covering the image. Use the same camera,
focus, zoom and resolution as tracking; lock focus where possible. Count INNER
corners, not squares. Example for 9x6 inner corners and 25 mm squares:

```powershell
.venv\Scripts\python calibrate.py --images "calibration_images/*.png" --columns 9 --rows 6 --square-m 0.025
.venv\Scripts\python app.py --calibration calibration.json --tag-size 0.080
```

The calibration JSON contains camera_matrix (3x3), dist_coeffs, image_size
([width,height]), RMS error and view count. Do not invent intrinsic values.
Check calibration against measured distances before relying on XYZ. Resolution
mismatch stops the app. Rejected poses omit XYZ but do not prevent pixel centering.

## 3. Arduino firmware and wiring

This package includes a NEW sketch and NEW serial protocol. It does not work
with the unmodified isaac879 repository firmware. Back up any existing sketch.
Install AccelStepper with Arduino IDE Library Manager, open
arduino_pan_tilt/arduino_pan_tilt.ino and upload to your classic Nano.

| Function | Default Nano pin |
|---|---|
| Pan STEP / DIR | D8 / D7 |
| Tilt STEP / DIR | D6 / D5 |
| Shared driver ENABLE, active LOW | D12 |
| Serial | USB, 115200 baud |

Confirm your actual wiring. Both driver grounds and Arduino ground must share a
reference. Driver VMOT uses motor power, not an Arduino output. Set driver current
correctly for your exact motors/modules. Never hot-plug motors. Use the power
arrangement discussed separately, including local supply capacitors.

Configure BOTH TMC2208 modules for **1/16 externally commanded microsteps** using
your module's MS configuration instructions. The sketch does not drive MS pins.
If using the original PCB with MS pins routed to Arduino, explicitly configure
those pins for your exact module before use; floating MS wiring is not acceptable.
Internal 256-step interpolation does not mean 256 external pulses per full step.
If microstepping differs, change MICROSTEPS in the sketch.

Default mechanics: 200 motor full steps/rev, pan ratio 144/17 and geared tilt
64/21. These are not the belt-tilt ratio in some repository versions.

### Establish zero and test directions

There is NO automatic homing in this version. With power removed and the camera
supported, place the mount at your chosen central pose. Apply power/reset the
Arduino without moving the mount. Startup step counts are now software zero.
Never manually move it afterward without resetting/re-establishing zero.
This assumption is essential: software limits cannot identify physical position.

Sketch defaults: pan +/-45 degrees and tilt +/-25 degrees relative to startup,
maximum 3 degrees/second, normal acceleration 10 degrees/second squared.
Set narrower limits if needed to avoid your frame, screen and cables. These are
not measured collision limits. Limit checks bound counted steps only; skipped
steps or manual motion invalidate position. No Hall sensors/endstops are used.

Close Arduino Serial Monitor before starting Python. Replace COM3 with your port:

```powershell
.venv\Scripts\python app.py --port COM3 --motors --zero-confirmed --tag-id 0
```

--zero-confirmed is your confirmation that startup zero was established; software
cannot verify it. Motors stay disabled until A is pressed with the chosen tag visible.
Briefly track a nearby tag offset to the right. If motion increases the image
error, press Space and restart with --pan-direction -1. Repeat vertically with
--tilt-direction -1 if needed. Do not assume default signs match your mount.
With correct signs, both errors should shrink toward zero. Keep the camera
supported on initial testing. Preview/detection requires no motor arming.

Add calibrated XYZ independently:

```powershell
.venv\Scripts\python app.py --port COM3 --motors --zero-confirmed --calibration calibration.json --tag-size 0.080
```

## Controller behavior and limits

Pixel errors are normalized by half-image dimensions. Proportional control uses
a 4% half-image deadband, gain 6 deg/s per normalized error, and max 3 deg/s.
Tune these in control.py. Commands are velocity setpoints, not absolute position.
Centering only controls direction of view: pan/tilt cannot set distance Z.
The overlay reports requested rates, not measured motor rates/positions.

The selected ID is fixed. Other tags are displayed but never substituted.
Tag loss immediately stops step pulses and disarms tracking (holding torque
remains). Reacquire the tag and press A to resume. Normal exit stops motion.
Firmware command watchdog is 350 ms: command loss abruptly stops pulses and
requires ARM again. After a long PC stall, press Space then A to resume. Abrupt
stops can lose steps; re-establish zero if position is uncertain. USB disconnect
that resets the Nano disables outputs; power loss can let the tilt axis drop.
Holding torque and software watchdogs are not physical brakes or an emergency stop.
Camera/driver serial calls can block briefly; the independent Arduino watchdog
handles absence of fresh commands. Stream buffering can introduce visual lag;
validate latency with your actual camera before increasing speed.

Protocol: READY APRILTAG_V1 on boot, ARM -> ARMED, V <pan_deg_s> <tilt_deg_s>,
STOP. Newline-terminated ASCII. Invalid commands stop/disarm. Default ENABLE
polarity assumes active-low drivers. No encoder feedback; no distance control.

## Import helpers directly

```python
from camera import open_camera, read_frame, close_camera
from vision import create_detector, detect_tags, select_tag
from control import tracking_rates

camera = open_camera()  # default computer webcam
try:
    detector = create_detector("36h11")
    frame = read_frame(camera)
    tag = select_tag(detect_tags(frame, detector), 0)
    if tag is not None:
        h, w = frame.shape[:2]
        pan_rate, tilt_rate = tracking_rates(tag.center, (w, h))
        print(tag.center, pan_rate, tilt_rate)  # No motor motion here.
finally:
    close_camera(camera)
```

## Verification

Run `python -m unittest -v`. Tests cover synthetic AprilTag detection, selected-ID
behavior, non-mutating overlays, recovery of known metric translation, control
deadband/direction/saturation, and serial command formatting without hardware.
No physical camera, Arduino, motors or calibration accuracy was tested here.
Compile/upload the sketch in Arduino IDE and bench-test before loading the mount.
This is prototype code, not validated for patient-facing or treatment use.

## References

* OpenCV tag detection: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
* OpenCV pose: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
* AccelStepper: https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
* pySerial: https://pyserial.readthedocs.io/en/latest/pyserial_api.html
* Mechanical reference: https://github.com/isaac879/Pan-Tilt-Mount
