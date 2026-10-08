"""Non-blocking handheld actions; never retry an uncertain hardware request."""
import json
import os
import threading
import time
from urllib.request import build_opener, ProxyHandler, Request

import rclpy
from rclpy.node import Node
from unitree_go.msg import WirelessController
from go2_start_button_sender.button_actions import ButtonGestures, RIGHT


class StartButtonSender(Node):
    def __init__(self):
        super().__init__('start_button_sender')
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.nav = str(param('nav_base_url', 'http://127.0.0.1:10088')).rstrip('/')
        self.arm = str(param('arm_base_url', 'http://192.168.123.100:18891')).rstrip('/')
        self.testclient = str(param('button_testclient_base_url', '')).rstrip('/')
        self.retract_pose = str(param('retract_pose', '/home/ubuntu/marm_demo_auto/src/control/d1_pose0.json'))
        mask = int(param('right_button_mask', RIGHT))
        if mask != RIGHT:
            self.get_logger().warning('Ignoring obsolete right_button_mask=%s; physical RIGHT is 8192' % mask)
        self.timeout = float(param('request_timeout', 180.0))
        self.gestures = ButtonGestures(
            float(param('double_click_window', 0.5)),
            float(param('long_press_duration', 2.0)),
            float(param('controller_stale_after', 0.5)))
        self.lock = threading.Lock()
        self.busy = False
        self.last_keys = None
        self.opener = build_opener(ProxyHandler({}))
        self.sub = self.create_subscription(WirelessController, '/wirelesscontroller', self.controller_callback, 10)
        self.get_logger().info('double-click UP=4096 forward, LEFT=32768 retract, DOWN=16384 return, RIGHT=8192 local workflow; START=4 grab; long-press START=testclient workflow')
        self.get_logger().info('nav=%s arm=%s testclient=%s' % (self.nav, self.arm, self.testclient or '<unset>'))

    def controller_callback(self, msg):
        keys, now = int(msg.keys), time.monotonic()
        if keys != self.last_keys:
            self.get_logger().info('controller keys=%d (0x%04x)' % (keys, keys))
            self.last_keys = keys
        with self.lock:
            if self.busy:
                self.gestures.observe_busy(keys, now)
                return
            action = self.gestures.feed(keys, now)
            if not action:
                return
            self.busy = True
        threading.Thread(target=self.run_action, args=(action,), daemon=True).start()

    def http(self, method, url, payload=None):
        data = None if payload is None else json.dumps(payload).encode('utf-8')
        req = Request(url, data=data, method=method, headers={'Content-Type': 'application/json'})
        with self.opener.open(req, timeout=self.timeout) as response:
            raw = response.read().decode('utf-8')
        try:
            result = json.loads(raw)
        except ValueError:
            if raw.strip() != 'OK':
                raise RuntimeError('Unexpected response: ' + raw[:200])
            return {'ok': True}
        if not isinstance(result, dict) or result.get('ok') is False or result.get('status') in ('fail', 'failed', 'error'):
            raise RuntimeError('Request failed: %s' % result)
        return result

    def wait_nav(self):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            state = self.http('GET', self.nav + '/api/v1/status')
            if not state.get('busy'):
                return
            time.sleep(0.25)
        raise TimeoutError('navigation still busy; inspect hardware before restarting node')

    def run_action(self, action):
        try:
            self.get_logger().info('action started: ' + action)
            if action in ('forward', 'return'):
                state = self.http('GET', self.nav + '/api/v1/status')
                if state.get('busy') or not state.get('pose_ready'):
                    self.get_logger().warning('Navigation busy or pose unavailable; action rejected')
                    return
                if action == 'return' and not state.get('original_position_recorded'):
                    self.get_logger().warning('No original position; forward first')
                    return
                endpoint = '/mock-move-mobile' if action == 'forward' else '/mock-back-original'
                self.http('POST', self.nav + endpoint, {})
                self.wait_nav()
            elif action == 'local_workflow':
                self.run_local_workflow()
            elif action == 'testclient_workflow':
                if not self.testclient:
                    raise RuntimeError('button_testclient_base_url is not configured')
                self.http('POST', self.testclient + '/api/v1/rabbit/start', {})
            else:
                payload = ({'target': 'white toy rabbit'} if action == 'grab' else
                           {'input': self.retract_pose, 'debug': False, 'log': True})
                self.http('POST', self.arm + '/api/v1/exec', payload)
                if action == 'retract':
                    # Clear residual grip fine-tune compensation written by the
                    # mobile tune, so the arm settles at the true standby pose
                    # instead of standby + delta and the next grab starts clean.
                    self.http('POST', self.arm + '/api/v1/fine-tune', {'x': 0, 'y': 0})
            self.get_logger().info('action completed: ' + action)
        except Exception as exc:
            # A failed network/business request must not disable the physical
            # controller.  Keep the node usable so the operator can choose a
            # recovery action (return/retract/etc.) without restarting ROS.
            self.get_logger().error('action %s failed: %s; controller remains available' % (action, exc))
        finally:
            with self.lock:
                self.busy = False

    def run_local_workflow(self):
        """Run the dog-local demo without going through core network."""
        state = self.http('GET', self.nav + '/api/v1/status')
        if state.get('busy') or not state.get('pose_ready'):
            raise RuntimeError('Navigation busy or pose unavailable')
        self.http('POST', self.nav + '/mock-move-mobile', {})
        self.wait_nav()
        self.http('POST', self.arm + '/api/v1/exec', {'target': 'white toy rabbit'})
        state = self.http('GET', self.nav + '/api/v1/status')
        if not state.get('original_position_recorded'):
            raise RuntimeError('Original position was not recorded')
        self.http('POST', self.nav + '/mock-back-original', {})
        self.wait_nav()


def main(args=None):
    rclpy.init(args=args)
    node = StartButtonSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
