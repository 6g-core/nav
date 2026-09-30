"""Pose and orientation helpers shared by navigation nodes."""

import math
import threading
import time

import numpy as np


def quaternion_to_euler(x, y, z, w):
    """Convert a quaternion to roll, pitch and yaw in radians."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=np.float64)


class PoseTracker:
    """Keep the latest robot pose and state in one small, testable object."""

    def __init__(self):
        self._lock = threading.Lock()
        self._rot = np.zeros(3, dtype=np.float64)
        self._odom = np.zeros(3, dtype=np.float64)
        self._state = None
        self._updated_at = None

    def is_fresh(self, max_age_s):
        """Whether a valid pose was received within the monotonic age limit."""
        with self._lock:
            return (self._updated_at is not None and
                    time.monotonic() - self._updated_at <= max_age_s)

    @property
    def rot(self):
        with self._lock:
            return self._rot.copy()

    @property
    def odom(self):
        with self._lock:
            return self._odom.copy()

    @property
    def state(self):
        with self._lock:
            return self._state

    def snapshot(self):
        """Return a consistent copy of rotation, position, and state."""
        with self._lock:
            return self._rot.copy(), self._odom.copy(), self._state

    def update_pose(self, msg):
        """Update pose from a ROS ``PoseStamped``-like message."""
        orientation = msg.pose.orientation
        rot = quaternion_to_euler(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        position = msg.pose.position
        odom = np.array(
            [position.x, position.y, position.z],
            dtype=np.float64,
        )
        quaternion = np.array([orientation.x, orientation.y,
                               orientation.z, orientation.w], dtype=np.float64)
        if (not np.all(np.isfinite(quaternion)) or
                not np.all(np.isfinite(rot)) or
                not np.all(np.isfinite(odom)) or
                np.linalg.norm(quaternion) == 0):
            with self._lock:
                self._updated_at = None
            return False
        with self._lock:
            self._rot = rot
            self._odom = odom
            self._updated_at = time.monotonic()
        return True

    def update_state(self, msg):
        """Update the state string from a ROS ``String``-like message."""
        with self._lock:
            self._state = msg.data
