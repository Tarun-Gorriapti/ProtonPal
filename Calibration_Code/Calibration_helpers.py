"""Public helper entry point; implementations remain in independent modules.

Use from the repository root:
    from Calibration_Code.Calibration_helpers import open_camera, detect_tags

Or run scripts directly inside Calibration_Code. Importing never opens hardware.
"""
if __package__:
    from .camera import open_camera, read_frame, close_camera, show_frame
    from .vision import (Tag, Calibration, Pose, create_detector, detect_tags,
                         select_tag, load_calibration, estimate_pose, draw_feedback)
    from .control import center_error, axis_rate, tracking_rates
    from .motors import (connect_arduino, arm_motors, send_velocity,
                         stop_motors, disconnect_arduino)
    from .calibrate import calibrate_images
else:
    from camera import open_camera, read_frame, close_camera, show_frame
    from vision import (Tag, Calibration, Pose, create_detector, detect_tags,
                        select_tag, load_calibration, estimate_pose, draw_feedback)
    from control import center_error, axis_rate, tracking_rates
    from motors import (connect_arduino, arm_motors, send_velocity,
                        stop_motors, disconnect_arduino)
    from calibrate import calibrate_images

__all__ = [
    'open_camera', 'read_frame', 'close_camera', 'show_frame',
    'Tag', 'Calibration', 'Pose', 'create_detector', 'detect_tags',
    'select_tag', 'load_calibration', 'estimate_pose', 'draw_feedback',
    'center_error', 'axis_rate', 'tracking_rates',
    'connect_arduino', 'arm_motors', 'send_velocity', 'stop_motors',
    'disconnect_arduino', 'calibrate_images',
]
