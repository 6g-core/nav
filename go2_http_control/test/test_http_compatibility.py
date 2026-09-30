"""Offline HTTP compatibility regressions using stubbed ROS and robot clients."""

import importlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np


class StubNode:
    """Provide only the ROS node methods needed by the action server."""

    def __init__(self, name):
        self.name = name
        self.logger = Mock()

    def create_subscription(self, **kwargs):
        return kwargs

    def get_logger(self):
        return self.logger


def load_server():
    """Import the real server in an isolated package with hardware stubs."""
    modules = {}
    definitions = {
        'rclpy': {},
        'rclpy.node': {'Node': StubNode},
        'clients': {},
        'clients.sport_client': {'SportClient': lambda node: Mock()},
        'clients.obstacles_avoid_client': {
            'ObstaclesAvoidClient': lambda node: Mock()},
        'clients.vui_client': {'VUIClient': lambda node: Mock()},
        'clients.utils': {'do_delay': lambda fn, delay: fn},
        'geometry_msgs': {},
        'geometry_msgs.msg': {'PoseStamped': object},
        'std_msgs': {},
        'std_msgs.msg': {'String': object},
    }
    for name, attributes in definitions.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        modules[name] = module

    package_dir = Path(__file__).resolve().parents[1] / 'http_control'
    package_name = '_nav_http_compatibility'
    spec = importlib.util.spec_from_file_location(
        package_name, package_dir / '__init__.py',
        submodule_search_locations=[str(package_dir)],
    )
    package = importlib.util.module_from_spec(spec)
    modules[package_name] = package
    with patch.dict(sys.modules, modules):
        spec.loader.exec_module(package)
        return importlib.import_module(package_name + '.http_control_server')


def pose_message(x=0.0, y=0.0):
    """Build a pose message without requiring ROS message packages."""
    return SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    ))


class HttpCompatibilityTest(unittest.TestCase):
    """Exercise actual routing, state tracking, and task submission."""

    def setUp(self):
        self.server = load_server()
        self.server.node = self.server.HttpActionServer()
        self.server.TASK_EXECUTOR = self.server.TaskExecutor(interval_s=0.001)
        # Route tests skip deliberate action delays, but retain cancellation
        # checks. Socket-level tests exercise the real interruptible waits.
        self.server._task_wait = lambda _: self.server._check_task()
        self.release_task = threading.Event()
        self.addCleanup(self.server.TASK_EXECUTOR.stop)
        self.addCleanup(self.release_task.set)

    def request(self, path, payload=None, method='POST'):
        body = json.dumps(payload or {}).encode('utf-8')
        handler = self.server.RequestHandler.__new__(self.server.RequestHandler)
        handler.path = path
        handler.headers = {'Content-Length': str(len(body))}
        handler.rfile = io.BytesIO(body)
        handler.wfile = io.BytesIO()
        handler.send_response = Mock()
        handler.send_header = Mock()
        handler.end_headers = Mock()
        errors = []

        def dispatch():
            try:
                getattr(handler, 'do_' + method)()
            except Exception as exc:
                errors.append(exc)

        thread = threading.Thread(target=dispatch, daemon=True)
        thread.start()
        thread.join(timeout=2.0)
        self.assertFalse(thread.is_alive(), 'request handler did not finish')
        if errors:
            raise errors[0]
        status = handler.send_response.call_args.args[0]
        return status, handler.wfile.getvalue()

    def stop_motion(self):
        status, _ = self.request('/v1/stop', {'timestamp': time.time()})
        self.assertEqual(status, 200)
        self.assertTrue(self.server.V1_STOP_EVENT.is_set())

    def wait_idle(self):
        deadline = time.monotonic() + 2.0
        while self.server.TASK_EXECUTOR.busy and time.monotonic() < deadline:
            threading.Event().wait(0.001)
        self.assertFalse(self.server.TASK_EXECUTOR.busy, 'task did not finish')

    def test_initialization_and_state_callbacks(self):
        self.assertIsNone(self.server.node.state)
        self.assertFalse(self.server.node.pose_ready)
        self.server.node.handle_state(SimpleNamespace(data='damping'))
        self.assertEqual(self.server.node.state, 'damping')
        self.server.node.handle_pose(pose_message(1.0, 2.0))
        self.assertTrue(self.server.node.pose_ready)
        np.testing.assert_array_equal(self.server.node.odom, [1.0, 2.0, 0.0])

    def test_status_aliases_share_pose_and_busy_state(self):
        for ready in (False, True):
            with self.subTest(pose_ready=ready):
                if ready:
                    self.server.node.handle_pose(pose_message())
                    self.server.original_pos = self.server.node.odom
                    self.server.TASK_EXECUTOR.submit(lambda: None)
                expected = {
                    'ok': True, 'busy': ready, 'pose_ready': ready,
                    'original_position_recorded': ready,
                }
                for path in ('/api/v1/status', '/v1/status',
                             '/api/v1/status?client=legacy'):
                    status, body = self.request(path, method='GET')
                    self.assertEqual(status, 200)
                    self.assertEqual(json.loads(body), expected)

    def test_stop_then_legacy_move_executes_real_motion_loop(self):
        self.stop_motion()
        self.server.TASK_EXECUTOR.start()
        self.server.node.handle_pose(pose_message())

        def advance(vx, vy):
            self.assertFalse(self.server.V1_STOP_EVENT.is_set())
            position = self.server.node.odom
            self.server.node.handle_pose(
                pose_message(position[0] + vx, position[1] + vy))

        with patch.object(self.server, '_motion_wait',
                          side_effect=lambda _, end: self.server._check_motion(end)), \
                patch.object(self.server, 'move', side_effect=advance) as move:
            status, body = self.request('/mock-move-mobile')
            self.assertEqual((status, body), (200, b'OK'))
            self.wait_idle()
            self.assertGreater(move.call_count, 0)
        self.assertGreaterEqual(self.server.node.odom[0], 0.8)
        np.testing.assert_array_equal(self.server.original_pos, [0.0, 0.0, 0.0])

    def test_all_legacy_async_routes_resume_after_stop(self):
        routes = ('/mock-move', '/mock-move-mobile', '/mock-back-mobile',
                  '/mock-back-original', '/mock-move1', '/mock-move2',
                  '/mock-move3', '/mock-move4', '/mock-back',
                  '/goleft', '/goright')
        self.server.original_pos = np.zeros(3)
        self.server.start_rot = np.zeros(3)
        self.server.TASK_EXECUTOR.start()
        observed = []

        def motion(*args):
            observed.append(self.server.V1_STOP_EVENT.is_set())

        for name in ('gx', 'gy', 'gx_without_refresh_start', 'change_yaw',
                     'turn_to_yaw', 'move_to_pos'):
            setattr(self.server, name, motion)
        for path in routes:
            with self.subTest(path=path):
                self.stop_motion()
                observed.clear()
                self.assertEqual(self.request(path), (200, b'OK'))
                self.wait_idle()
                self.assertTrue(observed)
                self.assertFalse(any(observed))

    def test_coarse_shift_waits_and_preserves_signed_distance_after_stop(self):
        self.server.TASK_EXECUTOR.start()
        for distance, velocity in ((0.3, -0.4), (-0.3, 0.4)):
            with self.subTest(distance=distance):
                self.stop_motion()
                observed = []

                def shift(length, speed):
                    observed.append((length, speed,
                                     self.server.V1_STOP_EVENT.is_set()))

                with patch.object(self.server, 'gy', side_effect=shift):
                    result = self.request('/coarse-shift', {'distance': distance})
                self.assertEqual(result, (200, b'OK'))
                self.assertEqual(observed, [(0.3, velocity, False)])
                self.wait_idle()

    def test_zero_distance_and_invalid_requests_do_not_resume(self):
        self.stop_motion()
        cases = [('/coarse-shift', {'distance': 0}, 200),
                 ('/coarse-shift', {'distance': 'invalid'}, 400),
                 ('/mock-back-original', {}, 400),
                 ('/v1/forward', {'timestamp': time.time()}, 400)]
        for path, payload, expected in cases:
            with self.subTest(path=path, payload=payload):
                self.assertEqual(self.request(path, payload)[0], expected)
                self.assertTrue(self.server.V1_STOP_EVENT.is_set())
                self.assertFalse(self.server.TASK_EXECUTOR.busy)

    def assert_busy_requests_preserve_stop(self):
        self.server.V1_STOP_EVENT.set()
        cases = [('/mock-move-mobile', {}, 400),
                 ('/coarse-shift', {'distance': 0.3}, 400),
                 ('/v1/forward', {'timestamp': time.time(),
                                  'distance_m': 0.3, 'speed_mps': 0.4}, 409)]
        for path, payload, expected in cases:
            with self.subTest(path=path):
                self.assertEqual(self.request(path, payload)[0], expected)
                self.assertTrue(self.server.V1_STOP_EVENT.is_set())

    def test_queued_task_rejections_preserve_stop(self):
        self.server.TASK_EXECUTOR.submit(lambda: None)
        self.assert_busy_requests_preserve_stop()

    def test_running_task_rejections_preserve_stop(self):
        started = threading.Event()

        def blocking_task():
            started.set()
            self.release_task.wait(timeout=3.0)

        self.server.TASK_EXECUTOR.submit(blocking_task)
        self.server.TASK_EXECUTOR.start()
        self.assertTrue(started.wait(timeout=1.0))
        self.assert_busy_requests_preserve_stop()

    def test_v1_action_still_resumes_after_stop(self):
        self.stop_motion()
        self.server.TASK_EXECUTOR.start()
        status, body = self.request('/v1/stand-up', {'timestamp': time.time()})
        self.assertEqual(status, 202)
        self.assertEqual(json.loads(body)['state'], 'queued')
        self.wait_idle()
        self.server.node.spc.stand_up.assert_called_once_with()
        self.assertFalse(self.server.V1_STOP_EVENT.is_set())

    def test_stop_cancels_queued_legacy_task(self):
        self.assertEqual(self.request('/mock-move-mobile'), (200, b'OK'))
        status, body = self.request('/v1/stop-move', {'timestamp': time.time()})
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)['cleared'], 1)
        self.assertFalse(self.server.TASK_EXECUTOR.busy)
        self.assertTrue(self.server.V1_STOP_EVENT.is_set())
        self.server.node.spc.stop_move.assert_called_once_with()

    def test_executor_prepares_accepted_task_before_worker_runs(self):
        prepared = threading.Event()
        observed = []
        self.server.TASK_EXECUTOR.start()
        self.assertTrue(self.server.TASK_EXECUTOR.submit(
            lambda: observed.append(prepared.is_set()), on_accept=prepared.set))
        self.wait_idle()
        self.assertEqual(observed, [True])

    def test_executor_does_not_prepare_rejected_task(self):
        self.server.TASK_EXECUTOR.submit(lambda: None)
        prepare = Mock()
        self.assertFalse(self.server.TASK_EXECUTOR.submit(
            lambda: None, on_accept=prepare))
        prepare.assert_not_called()


if __name__ == '__main__':
    unittest.main()
