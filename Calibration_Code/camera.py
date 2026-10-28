"""Camera helpers. No detector, controller or serial dependencies."""
import cv2


def open_camera(source=0, width=1280, height=720):
    """Open local camera index (0 = default) or stream URL."""
    camera = cv2.VideoCapture(source)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"Cannot open camera {source!r}")
    if isinstance(source, int):
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return camera


def read_frame(camera):
    ok, frame = camera.read()
    if not ok or frame is None:
        raise RuntimeError("Camera frame read failed")
    return frame


def close_camera(camera):
    if camera is not None:
        camera.release()


def show_frame(frame, window="AprilTag camera"):
    """Display image and return pressed key code. No detection performed."""
    cv2.imshow(window, frame)
    return cv2.waitKey(1) & 0xFF
