"""Serial helper methods for INCLUDED firmware, not the original repo protocol."""
import math
import time
import serial


def connect_arduino(port, baud=115200, ready_timeout=5):
    connection = serial.Serial(port, baud, timeout=.1, write_timeout=.2)
    deadline = time.monotonic() + ready_timeout
    try:
        while time.monotonic() < deadline:
            if connection.readline().strip() == b"READY APRILTAG_V1":
                return connection
        raise TimeoutError("No firmware greeting; upload included sketch and close Serial Monitor")
    except Exception:
        connection.close()
        raise


def arm_motors(connection):
    """Enable holding torque; stationary startup pose is software zero."""
    connection.reset_input_buffer()
    connection.write(b"ARM\n")
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        if connection.readline().strip() == b"ARMED":
            return
    raise TimeoutError("Arduino did not acknowledge ARM")


def send_velocity(connection, pan_deg_s, tilt_deg_s):
    """Send rates only; camera position and tag detection are irrelevant here."""
    if not all(math.isfinite(v) and abs(v) <= 3 for v in (pan_deg_s, tilt_deg_s)):
        raise ValueError("Rates must be finite and within +/-3 deg/s")
    connection.write(f"V {pan_deg_s:.4f} {tilt_deg_s:.4f}\n".encode("ascii"))


def stop_motors(connection):
    """Immediate pulse stop, keeps holding torque, disarms tracking."""
    connection.write(b"STOP\n")


def disconnect_arduino(connection):
    if connection is not None:
        try:
            stop_motors(connection)
            connection.flush()
        finally:
            connection.close()
