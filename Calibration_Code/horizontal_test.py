"""Basic single-AprilTag pan test. Place beside camera.py/vision.py/motors.py.

Uses the included APRILTAG_V1 Arduino sketch. No calibration or tag size needed.
Center mount BEFORE Arduino reset; software zero/limits depend on that position.
"""
import argparse
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
    parser = argparse.ArgumentParser(description="Camera tracking + manual pan jog test")
    parser.add_argument('--motors', action='store_true')
    parser.add_argument('--port', default=PORT)
    args = parser.parse_args()
    camera = arduino = None
    jog_until = 0.0
    jog_speed = 0.0
    tracking = False
    last_send = last_frame = time.monotonic()
    detector = create_detector("36h11")
    try:
        camera = open_camera(CAMERA)
        if USE_MOTORS or args.motors:
            print(f"Opening {args.port} at 115200; waiting for ESP32 greeting...")
            arduino = connect_arduino(args.port)
            print("CONNECTED: received READY APRILTAG_V1. Press C to check ARM acknowledgment.")
        else:
            print("CAMERA ONLY: run with --motors --port COM9 to enable serial control.")
        print("Click camera window. LEFT/RIGHT (or J/L): brief jog; A: tag tracking; Space: stop; Q: quit.")
        while True:
            frame = read_frame(camera)
            now = time.monotonic()
            if (tracking or jog_until > 0) and now - last_frame > 0.25:
                stop_motors(arduino)
                tracking = False
                jog_until = 0.0
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
            if jog_until > 0 and now >= jog_until:
                stop_motors(arduino)
                jog_until = 0.0
                print("Jog stopped")
            if jog_until > now and now - last_send >= 0.05:
                send_velocity(arduino, jog_speed, 0.0)
                last_send = now
            if tracking and now - last_send >= 0.05:
                send_velocity(arduino, pan_speed, 0.0)  # Tilt ALWAYS stationary.
                last_send = now
            status = "JOGGING" if jog_until > now else ("TRACKING" if tracking else "PAUSED")
            cv2.putText(frame, f"{status} | Arrows/J/L: jog | C: check | A: track | SPACE: stop | Q: quit",
                        (10,25), cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,255), 2)
            cv2.imshow("Horizontal test", frame)
            key = cv2.waitKeyEx(1)
            if key == ord('q') or cv2.getWindowProperty('Horizontal test',cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord(' ') or (key == ord('a') and tracking):
                if arduino is not None: stop_motors(arduino)
                tracking = False
                jog_until = 0.0
            elif key in (2424832, 2555904, 65361, 65363, 81, 83, ord('j'), ord('l')):
                if arduino is None:
                    print("No serial connection: restart with --motors --port YOUR_PORT")
                    continue
                tracking = False
                # Each key event requests only 150 ms of movement.
                # OS key-repeat may produce successive short jogs.
                stop_motors(arduino)
                arm_motors(arduino)
                left = key in (2424832, 65361, 81, ord('j'))
                jog_speed = (-MAX_SPEED if left else MAX_SPEED) * PAN_DIRECTION
                send_velocity(arduino, jog_speed, 0.0)
                last_frame = last_send = time.monotonic()
                jog_until = last_send + 0.15
                print(f"ARM acknowledged; sent pan={jog_speed:+.2f} deg/s, tilt=0")
            elif key == ord('c'):
                if arduino is not None:
                    tracking = False
                    jog_until = 0.0
                    stop_motors(arduino)
                    arm_motors(arduino)
                    stop_motors(arduino)
                    last_frame = time.monotonic()
                    print("PASS: ESP32 acknowledged ARM; STOP sent. This verifies serial/firmware, not motor motion.")
                else:
                    print("CAMERA ONLY: enable --motors and specify --port")
            elif key == ord('a') and arduino is not None and tag is not None:
                jog_until = 0.0
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
