import unittest
import math
import cv2
import numpy as np
from control import axis_rate, center_error, tracking_rates
from vision import Calibration, create_detector, detect_tags, estimate_pose, select_tag, draw_feedback
from motors import send_velocity, stop_motors


class MathTests(unittest.TestCase):
    def test_center_and_limits(self):
        self.assertEqual(center_error((320, 240), (640, 480)), (0, 0))
        self.assertEqual(tracking_rates((320, 240), (640, 480)), (0, 0))
        self.assertEqual(axis_rate(.02), 0)
        self.assertEqual(axis_rate(100), 3)
        self.assertEqual(axis_rate(-100), -3)
        self.assertGreater(axis_rate(.5), 0)
        self.assertLess(axis_rate(.5, direction=-1), 0)
        with self.assertRaises(ValueError):
            axis_rate(float('nan'))


class VisionTests(unittest.TestCase):
    def test_detection_and_annotation(self):
        detector = create_detector()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        marker = cv2.aruco.generateImageMarker(dictionary, 7, 200)
        gray = np.full((480, 640), 255, np.uint8)
        gray[140:340, 220:420] = marker
        frame = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        original = frame.copy()
        tags = detect_tags(frame, detector)
        tag = select_tag(tags, 7)
        self.assertIsNotNone(tag)
        self.assertIsNone(select_tag(tags, 0))
        np.testing.assert_allclose(tag.center, [319.5, 239.5], atol=1)
        out = draw_feedback(frame, tags, 7)
        np.testing.assert_array_equal(frame, original)
        self.assertFalse(np.array_equal(out, frame))
        self.assertEqual(detect_tags(np.zeros_like(frame), detector), [])

    def test_metric_pose(self):
        k = np.array([[800., 0, 320], [0, 800, 240], [0, 0, 1]])
        cal = Calibration(k, np.zeros(5), 640, 480)
        s = .04
        obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]])
        rvec = np.array([math.pi - .2, .05, .03])
        tvec = np.array([.03, -.02, .8])
        image, _ = cv2.projectPoints(obj, rvec, tvec, k, cal.distortion)
        pose = estimate_pose(image, .08, cal)
        self.assertIsNotNone(pose)
        np.testing.assert_allclose(pose.tvec.ravel(), tvec, atol=1e-5)
        self.assertLess(pose.reprojection_error_px, .01)
        with self.assertRaises(ValueError):
            estimate_pose(image, 0, cal)


class SerialTests(unittest.TestCase):
    def test_protocol_without_hardware(self):
        class FakeSerial:
            def __init__(self): self.commands = []
            def write(self, data): self.commands.append(data)
        port = FakeSerial()
        send_velocity(port, 1.25, -2)
        stop_motors(port)
        self.assertEqual(port.commands, [b'V 1.2500 -2.0000\n', b'STOP\n'])
        with self.assertRaises(ValueError): send_velocity(port, 4, 0)
        with self.assertRaises(ValueError): send_velocity(port, float('nan'), 0)


if __name__ == '__main__':
    unittest.main()
