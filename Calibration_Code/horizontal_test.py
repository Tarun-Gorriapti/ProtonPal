"""Basic single-AprilTag pan test. Place beside camera.py/vision.py/motors.py.

Uses the included APRILTAG_V1 Arduino sketch. No calibration or tag size needed.
Center mount BEFORE Arduino reset; software zero/limits depend on that position.
"""
import time
import cv2
from camera import open_camera, read_frame, close_camera
from vision import create_detector, detect_tags
from motors import connect_arduino, arm_motors, send_velocity, stop_motors, disconnect_arduino

# EDIT THESE SETTINGS
USE_MOTORS = False       # First check detection; then change to True.
PORT = "COM3"           # Your Arduino port. Close Arduino Serial Monitor.
CAMERA = 0              # Default computer webcam.
TAG_ID = 0              # Show AprilTag family 36h11, ID 0 on your phone.
PAN_DIRECTION = 1       # Change to -1 if it moves AWAY from the tag.
DEADBAND_PX = 20         # Stop when within 20 pixels of horizontal center.
GAIN = 0.004            # Degrees/second per pixel of horizontal error.
MAX_SPEED = 0.8         # Slow first test, degrees/second at geared output.


def main():
    camera = arduino = None
    tracking = False
    last_send = last_frame = time.monotonic()
    detector = create_detector("36h11")
    try:
        camera = open_camera(CAMERA)
        if USE_MOTORS:
            arduino = connect_arduino(PORT)
        while True:
            frame = read_frame(camera)
            now = time.monotonic()
            if tracking and now - last_frame > 0.25:
                stop_motors(arduino)
                tracking = False  # Press A again after a long camera/PC stall.
            last_frame = now
            height, width = frame.shape[:2]
            cx, cy = width / 2, height / 2
            tags = detect_tags(frame, detector)
            matches = [t for t in tags if t.tag_id == TAG_ID]
            tag = matches[0] if len(matches) == 1 else None
            cv2.drawMarker(frame, (int(cx), int(cy)), (255,255,0), cv2.MARKER_CROSS, 25, 2)
            pan_speed = 0.0
            if tag is not None:
                x, y = tag.center
                error_x, error_y = x - cx, y - cy
                cv2.polylines(frame, [tag.corners.astype('int32')], True, (0,255,0), 2)
                cv2.line(frame, (int(cx),int(cy)), (int(x),int(y)), (0,255,0), 2)
                cv2.putText(frame, f"x={x:.0f} y={y:.0f} px | dx={error_x:+.0f} dy={error_y:+.0f}",
                            (10,60), cv2.FONT_HERSHEY_SIMPLEX, .6, (0,255,0), 2)
                if abs(error_x) > DEADBAND_PX:
                    pan_speed = PAN_DIRECTION * max(-MAX_SPEED, min(MAX_SPEED, GAIN*error_x))
            elif tracking:
                stop_motors(arduino)
                tracking = False  # No automatic resume on tag reacquisition.
            if tracking and now - last_send >= 0.05:
                send_velocity(arduino, pan_speed, 0.0)  # Tilt ALWAYS stationary.
                last_send = now
            status = "TRACKING" if tracking else "PAUSED"
            cv2.putText(frame, f"{status} | A: start/pause | SPACE: stop | Q: quit",
                        (10,25), cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,255), 2)
            cv2.imshow("Horizontal test", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or cv2.getWindowProperty('Horizontal test',cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord(' ') or (key == ord('a') and tracking):
                if arduino is not None: stop_motors(arduino)
                tracking = False
            elif key == ord('a') and arduino is not None and tag is not None:
                arm_motors(arduino)
                tracking = True
                last_frame = time.monotonic()
                last_send = 0.0
    finally:
        try:
            disconnect_arduino(arduino)
        finally:
            close_camera(camera)
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
