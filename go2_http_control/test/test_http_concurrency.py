"""Regression tests for cancellation and bounded waits over real local HTTP."""

import http.client
import json
import threading
import time
import unittest
from unittest.mock import Mock, patch

from test_http_compatibility import load_server, pose_message


class HttpConcurrencyTest(unittest.TestCase):
    """Run the threaded HTTP server with real queues and stubbed robot I/O."""

    def setUp(self):
        self.server = load_server()
        self.server.node = self.server.HttpActionServer()
        self.server.node.handle_pose(pose_message())
        self.server.TASK_EXECUTOR = self.server.TaskExecutor(interval_s=0.001)
        self.release = threading.Event()
        self.http = self.server.ThreadingHTTPServer(
            ('127.0.0.1', 0), self.server.RequestHandler)
        self.http_thread = threading.Thread(
            target=self.http.serve_forever,
            kwargs={'poll_interval': 0.01}, daemon=True)
        self.http_thread.start()
        self.clients = []

    def tearDown(self):
        self.release.set()
        self.server._stop_tasks()
        self.server.TASK_EXECUTOR.stop()
        for thread in self.clients:
            thread.join(timeout=2.0)
        self.http.shutdown()
        self.http.server_close()
        self.http_thread.join(timeout=1.0)

    def request(self, path, payload=None, method='POST'):
        connection = http.client.HTTPConnection(
            '127.0.0.1', self.http.server_port, timeout=2.0)
        try:
            connection.request(method, path, json.dumps(payload or {}),
                               {'Content-Type': 'application/json'})
            response = connection.getresponse()
            return response.status, response.read().decode('utf-8')
        finally:
            connection.close()

    def start_request(self, path, payload=None):
        result = {}

        def run():
            try:
                result['response'] = self.request(path, payload)
            except Exception as exc:
                result['error'] = exc

        thread = threading.Thread(target=run, daemon=True)
        self.clients.append(thread)
        thread.start()
        return thread, result

    def finish_request(self, thread, result, status):
        thread.join(timeout=1.0)
        self.assertFalse(thread.is_alive(), 'HTTP waiter was not released')
        if 'error' in result:
            raise result['error']
        self.assertEqual(result['response'][0], status, result)

    def wait_for(self, predicate):
        deadline = time.monotonic() + 1.0
        while not predicate() and time.monotonic() < deadline:
            threading.Event().wait(0.002)
        self.assertTrue(predicate())

    def test_running_shift_does_not_block_status_or_stop(self):
        self.server.TASK_EXECUTOR.start()
        thread, result = self.start_request('/coarse-shift', {'distance': 0.3})
        self.wait_for(lambda: self.server.node.spc.move.called)
        self.assertTrue(thread.is_alive())
        status, body = self.request('/api/v1/status', method='GET')
        self.assertEqual(status, 200)
        self.assertTrue(json.loads(body)['busy'])
        status, _ = self.request('/v1/stop', {'timestamp': time.time()})
        self.assertEqual(status, 200)
        self.finish_request(thread, result, 409)
        self.wait_for(lambda: not self.server.TASK_EXECUTOR.busy)

    def test_stop_wakes_waiter_for_task_that_never_started(self):
        thread, result = self.start_request('/coarse-shift', {'distance': 0.3})
        self.wait_for(lambda: self.server.TASK_EXECUTOR.busy)
        status, body = self.request('/v1/stop', {'timestamp': time.time()})
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)['cleared'], 1)
        self.finish_request(thread, result, 409)
        self.assertFalse(self.server.TASK_EXECUTOR.busy)
        self.server.node.spc.move.assert_not_called()

    def test_queue_wait_timeout_cancels_task_without_worker(self):
        self.server.SYNC_WAIT_TIMEOUT_S = 0.05
        status, body = self.request('/coarse-shift', {'distance': 0.3})
        self.assertEqual(status, 504)
        self.assertIn('Timed out', body)
        self.assertFalse(self.server.TASK_EXECUTOR.busy)
        self.assertTrue(self.server.V1_STOP_EVENT.is_set())
        self.server.TASK_EXECUTOR.start()
        self.server.node.spc.move.assert_not_called()

    def test_wait_timeout_keeps_unfinished_task_busy(self):
        self.server.SYNC_WAIT_TIMEOUT_S = 0.05
        started = threading.Event()

        def blocked_shift(*args):
            started.set()
            self.release.wait(timeout=2.0)

        self.server.gy = blocked_shift
        self.server.TASK_EXECUTOR.start()
        thread, result = self.start_request('/coarse-shift', {'distance': 0.3})
        self.assertTrue(started.wait(timeout=1.0))
        self.finish_request(thread, result, 504)
        self.assertTrue(self.server.TASK_EXECUTOR.busy)
        self.assertEqual(self.request('/mock-move-mobile')[0], 400)
        self.assertTrue(self.server.V1_STOP_EVENT.is_set())
        self.release.set()
        self.wait_for(lambda: not self.server.TASK_EXECUTOR.busy)

    def test_stale_pose_aborts_motion_and_returns_503(self):
        self.server.POSE_MAX_AGE_S = 0.05
        self.server.TASK_EXECUTOR.start()
        status, body = self.request('/coarse-shift', {'distance': 0.3})
        self.assertEqual(status, 503)
        self.assertIn('pose', body)
        self.wait_for(lambda: not self.server.TASK_EXECUTOR.busy)
        self.assertTrue(self.server.node.spc.stop_move.called)
        status, body = self.request('/v1/status', method='GET')
        self.assertFalse(json.loads(body)['pose_ready'])

    def test_fresh_but_stationary_pose_hits_motion_deadline(self):
        self.server.MOTION_TIMEOUT_S = 0.05
        self.server.TASK_EXECUTOR.start()
        status, body = self.request('/coarse-shift', {'distance': 0.3})
        self.assertEqual(status, 504)
        self.assertIn('Movement timed out', body)
        self.wait_for(lambda: not self.server.TASK_EXECUTOR.busy)
        self.assertTrue(self.server.node.spc.stop_move.called)

    def test_missing_pose_and_nonfinite_distance_are_rejected(self):
        self.server.node = self.server.HttpActionServer()
        self.server.TASK_EXECUTOR.start()
        self.assertEqual(self.request('/coarse-shift', {'distance': 0.3})[0], 503)
        self.server.node.spc.move.assert_not_called()
        for distance in ('NaN', 'Infinity', '-Infinity'):
            with self.subTest(distance=distance):
                self.assertEqual(
                    self.request('/coarse-shift', {'distance': distance})[0], 400)

    def test_movement_success_preserves_synchronous_200(self):
        def move(vx, vy, vyaw):
            position = self.server.node.odom
            self.server.node.handle_pose(
                pose_message(position[0] + vx, position[1] + vy))

        self.server.node.spc.move.side_effect = move
        self.server.TASK_EXECUTOR.start()
        status, body = self.request('/coarse-shift', {'distance': 0.3})
        self.assertEqual((status, body), (200, 'OK'))
        self.assertFalse(self.server.TASK_EXECUTOR.busy)
        self.assertLessEqual(self.server.node.odom[1], -0.3)

    def test_stop_aborts_remaining_steps_of_route(self):
        self.server.TASK_EXECUTOR.start()
        later_step = Mock()
        with patch.object(self.server, 'change_yaw', later_step):
            self.assertEqual(self.request('/mock-move')[0], 200)
            self.wait_for(lambda: self.server.node.spc.move.called)
            self.assertEqual(self.request('/v1/stop',
                                          {'timestamp': time.time()})[0], 200)
            self.wait_for(lambda: not self.server.TASK_EXECUTOR.busy)
            later_step.assert_not_called()

    def test_admission_waits_until_stop_command_is_published(self):
        stop_started = threading.Event()

        def slow_stop():
            stop_started.set()
            self.release.wait(timeout=1.0)

        self.server.node.spc.stop_move.side_effect = slow_stop
        stop_thread, stop_result = self.start_request(
            '/v1/stop', {'timestamp': time.time()})
        self.assertTrue(stop_started.wait(timeout=1.0))
        move_thread, move_result = self.start_request('/mock-move-mobile')
        # The stop owns the command lock; the next task cannot clear its flag.
        self.assertTrue(self.server.V1_STOP_EVENT.is_set())
        self.assertFalse(self.server.TASK_EXECUTOR.busy)
        self.assertEqual(self.request('/v1/status', method='GET')[0], 200)
        self.release.set()
        self.finish_request(stop_thread, stop_result, 200)
        self.finish_request(move_thread, move_result, 200)
        self.assertFalse(self.server.V1_STOP_EVENT.is_set())

    def test_late_timeout_does_not_cancel_successor(self):
        self.server.TASK_EXECUTOR.start()
        first = self.server._submit_task(lambda: None)
        self.assertTrue(first.done.wait(timeout=1.0))
        started = threading.Event()

        def successor():
            started.set()
            self.release.wait(timeout=1.0)

        second = self.server._submit_task(successor)
        self.assertIsNotNone(second)
        self.assertTrue(started.wait(timeout=1.0))
        self.server.node.spc.stop_move.reset_mock()
        self.assertIsNone(self.server._stop_tasks(
            first, self.server.MotionTimeout('late timeout')))
        self.assertFalse(self.server.V1_STOP_EVENT.is_set())
        self.assertFalse(second.done.is_set())
        self.server.node.spc.stop_move.assert_not_called()

    def test_return_to_origin_deadline_reaches_nested_motion(self):
        self.server.node.handle_pose(pose_message(1.0, 0.0))
        with self.assertRaises(self.server.MotionTimeout):
            self.server._run_task(
                lambda: self.server.move_to_pos([0.0, 0.0], 0.4, timeout_s=0.03))
        self.assertTrue(self.server.node.spc.stop_move.called)

    def test_invalid_pose_does_not_mark_tracker_ready(self):
        self.server.node.handle_pose(pose_message(float('nan'), 0.0))
        self.assertFalse(self.server.node.pose_ready)
        self.server.node.handle_pose(pose_message())
        self.assertTrue(self.server.node.pose_ready)

    def test_simultaneous_legacy_requests_accept_exactly_one_task(self):
        barrier = threading.Barrier(6)
        responses = []

        def submit():
            barrier.wait(timeout=1.0)
            responses.append(self.request('/mock-move-mobile')[0])

        threads = [threading.Thread(target=submit) for _ in range(6)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=2.0)
            self.assertFalse(thread.is_alive())
        self.assertEqual(sorted(responses), [200, 400, 400, 400, 400, 400])

    def test_task_deadline_is_enforced_inside_motion(self):
        self.server.TASK_TIMEOUT_S = 0.03
        self.server.TASK_EXECUTOR.start()
        status, body = self.request('/coarse-shift', {'distance': 0.3})
        self.assertEqual(status, 504)
        self.assertIn('Task execution timed out', body)

    def test_task_failure_returns_500_and_worker_remains_usable(self):
        self.server.node.spc.move.side_effect = RuntimeError('client failed')
        self.server.TASK_EXECUTOR.start()
        status, body = self.request('/coarse-shift', {'distance': 0.3})
        self.assertEqual(status, 500)
        self.assertIn('client failed', body)
        handle = self.server._submit_task(lambda: None)
        self.assertIsNotNone(handle)
        self.assertTrue(handle.done.wait(timeout=1.0))
        self.assertIsNone(handle.error)

    def test_clear_pending_also_completes_cancelled_handles(self):
        handle = self.server.TASK_EXECUTOR.submit_handle(lambda: None)
        self.assertEqual(self.server.TASK_EXECUTOR.clear_pending(), 1)
        self.assertTrue(handle.done.is_set())
        self.assertIsInstance(handle.error, self.server.TaskCancelled)


if __name__ == '__main__':
    unittest.main()
