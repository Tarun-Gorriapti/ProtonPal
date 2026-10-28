"""Pure control math: no hardware, OpenCV or serial dependencies."""
import math


def center_error(center_px, image_size):
    """Normalized offsets: positive right/down; image_size = (width,height)."""
    w, h = image_size
    if w <= 0 or h <= 0:
        raise ValueError("Image size must be positive")
    return (center_px[0] - w / 2) / (w / 2), (center_px[1] - h / 2) / (h / 2)


def axis_rate(error, gain=6.0, deadband=.04, max_rate=3.0, direction=1):
    """Return bounded deg/s. direction is chosen by actual wiring/mount."""
    if not all(math.isfinite(x) for x in (error, gain, deadband, max_rate, direction)):
        raise ValueError("Control inputs must be finite")
    if gain < 0 or max_rate <= 0 or not 0 <= deadband < 1 or direction not in (-1, 1):
        raise ValueError("Invalid control settings")
    if abs(error) <= deadband:
        return 0.0
    adjusted = math.copysign(abs(error) - deadband, error)
    return direction * max(-max_rate, min(max_rate, gain * adjusted))


def tracking_rates(center_px, image_size, pan_direction=1, tilt_direction=1):
    ex, ey = center_error(center_px, image_size)
    return (axis_rate(ex, direction=pan_direction), axis_rate(ey, direction=tilt_direction))
