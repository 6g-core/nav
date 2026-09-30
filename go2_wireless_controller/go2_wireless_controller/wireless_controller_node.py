"""Translate wireless-controller gestures into navigation and arm requests."""

import json
import threading
import time
from urllib.request import Request, build_opener, ProxyHandler

import rclpy
from rclpy.node import Node
from unitree_go.msg import WirelessController

from .button_actions import ButtonGestures


class WirelessControllerNode(Node):
    def __init__(self):
        super().__init__("wireless_controller")

        def parameter(name, default):
            return self.declare_parameter(name, default).value

        self.nav_base_url = str(parameter("nav_base_url", "http://127.0.0.1:10088")).rstrip("/")
        self.arm_base_url = str(parameter("arm_base_url", "http://192.168.123.100:18891")).rstrip("/")
        self.testclient_base_url = str(
            parameter(
                "button_testclient_base_url",
                parameter("testclient_base_url", ""),
            )
        ).rstrip("/")
        self.retract_pose = str(parameter("retract_pose", "/home/ubuntu/marm_demo_auto/src/control/d1_pose0.json"))
        self.request_timeout = float(parameter("request_timeout", 180.0))
        self.gestures = ButtonGestures(
            float(parameter("double_click_window", 0.5)),
            float(parameter("long_press_duration", 2.0)),
            float(parameter("controller_stale_after", 0.5)),
        )
        self._lock = threading.Lock()
        self._busy = False
        self._last_keys = None
        self._http = build_opener(ProxyHandler({}))
        self.create_subscription(WirelessController, "/wirelesscontroller", self.controller_callback, 10)
        self.get_logger().info("buttons: UP forward, DOWN return, LEFT retract, RIGHT local workflow, START grab")
        self.get_logger().info(f"nav={self.nav_base_url} arm={self.arm_base_url} testclient={self.testclient_base_url or '<unset>'}")

    def controller_callback(self, msg):
        keys = int(msg.keys)
        now = time.monotonic()
        if keys != self._last_keys:
            self.get_logger().info(f"controller keys={keys} (0x{keys:04x})")
            self._last_keys = keys
        with self._lock:
            if self._busy:
                self.gestures.observe_busy(keys, now)
                return
            action = self.gestures.feed(keys, now)
            if not action:
                return
            self._busy = True
        threading.Thread(target=self.run_action, args=(action,), daemon=True).start()

    def http(self, method, url, payload=None):
        data = None if payload is None else json.dumps(payload).encode("utf-8")
        request = Request(url, data=data, method=method, headers={"Content-Type": "application/json"})
        with self._http.open(request, timeout=self.request_timeout) as response:
            if response.status >= 400:
                raise RuntimeError(f"HTTP request failed: {response.status}")
            raw = response.read().decode("utf-8")
        try:
            result = json.loads(raw)
        except ValueError:
            if raw.strip() in {"OK", ""}:
                return {"ok": True}
            raise RuntimeError(f"unexpected response: {raw[:200]}")
        if not isinstance(result, dict):
            raise RuntimeError(f"response must be an object: {result}")
        if result.get("ok") is False or result.get("status") in {
            "fail", "failed", "error"
        }:
            raise RuntimeError(f"request failed: {result}")
        return result

    def navigation_status(self):
        return self.http("GET", self.nav_base_url + "/v1/status")

    def require_navigation_ready(self, require_original=False):
        state = self.navigation_status()
        if state.get("busy"):
            raise RuntimeError("navigation is busy")
        if not state.get("pose_ready"):
            raise RuntimeError("navigation pose is not ready")
        if require_original and not state.get("original_position_recorded"):
            raise RuntimeError("navigation origin has not been recorded")

    def wait_nav(self):
        deadline = time.monotonic() + self.request_timeout
        while time.monotonic() < deadline:
            state = self.navigation_status()
            if not state.get("busy"):
                return
            time.sleep(0.25)
        raise TimeoutError("navigation is still busy")

    def run_action(self, action):
        try:
            self.get_logger().info(f"action started: {action}")
            if action == "forward":
                self.require_navigation_ready()
                self.http("POST", self.nav_base_url + "/mock-move-mobile", {})
                self.wait_nav()
            elif action == "return":
                self.require_navigation_ready(require_original=True)
                self.http("POST", self.nav_base_url + "/mock-back-original", {})
                self.wait_nav()
            elif action == "grab":
                self.http("POST", self.arm_base_url + "/api/v1/exec", {"target": "white toy rabbit"})
            elif action == "retract":
                self.http("POST", self.arm_base_url + "/api/v1/exec", {"input": self.retract_pose, "debug": False, "log": True})
            elif action == "local_workflow":
                self.run_local_workflow()
            elif action == "testclient_workflow":
                if not self.testclient_base_url:
                    raise RuntimeError("testclient_base_url is not configured")
                self.http("POST", self.testclient_base_url + "/api/v1/rabbit/start", {})
            self.get_logger().info(f"action completed: {action}")
        except Exception as exc:
            self.get_logger().error(f"action {action} failed: {exc}")
        finally:
            with self._lock:
                self._busy = False

    def run_local_workflow(self):
        self.require_navigation_ready()
        self.http("POST", self.nav_base_url + "/mock-move-mobile", {})
        self.wait_nav()
        self.http("POST", self.arm_base_url + "/api/v1/exec", {"target": "white toy rabbit"})
        self.require_navigation_ready(require_original=True)
        self.http("POST", self.nav_base_url + "/mock-back-original", {})
        self.wait_nav()


def main(args=None):
    rclpy.init(args=args)
    node = WirelessControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
