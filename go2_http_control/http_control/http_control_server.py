import rclpy
from rclpy.node import Node
import time
import math
import threading
import json
import uuid
from datetime import datetime
from urllib.parse import urlsplit

from clients.obstacles_avoid_client import ObstaclesAvoidClient
from clients.sport_client import SportClient
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from geometry_msgs.msg import PoseStamped
import numpy as np
from clients.vui_client import VUIClient
from std_msgs.msg import String
from .common import (
    PoseTracker, TaskCancelled, TaskExecutor, read_request_body, parse_json_body,
)
from .common import (
    send_error,
    send_json,
    send_method_not_allowed,
    send_ok,
    send_text,
    validate_timestamp,
)
node = None
start_rot = None
original_pos = None

HTTP_ACTION_PORT = 10088
POSE_MAX_AGE_S = 2.0
MOTION_TIMEOUT_S = 30.0
TASK_TIMEOUT_S = 120.0
SYNC_WAIT_TIMEOUT_S = 35.0


def nav_log(level, message):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    formatted = f"[nav] {timestamp} {message}"
    if node is None:
        print(formatted)
        return
    logger = node.get_logger()
    if level == "error":
        logger.error(formatted)
    elif level == "warning":
        logger.warning(formatted)
    else:
        logger.info(formatted)


def nav_info(message):
    nav_log("info", message)


def nav_error(message):
    nav_log("error", message)


class HttpActionServer(Node):
    def __init__(self):
        super().__init__('http_control_server')
        self.pose_tracker = PoseTracker()

        self.spc = SportClient(self)
        self.oac = ObstaclesAvoidClient(self)
        self.vuc = VUIClient(self)

        # 并非定时器，单次sub
        self.pose_sub = self.create_subscription(
                    msg_type=PoseStamped,
                    topic='/utlidar/robot_pose',
                    callback=self.handle_pose,
                    qos_profile=10
                )
        
        self.state_sub = self.create_subscription(
                    msg_type=String,
                    topic='/ut/state',
                    callback=self.handle_state,
                    qos_profile=10
        )

    def handle_state(self, msg: String):
        self.pose_tracker.update_state(msg)
    
    def handle_pose(self, msg: PoseStamped):
        self.pose_tracker.update_pose(msg)

    @property
    def pose_ready(self):
        return self.pose_tracker.is_fresh(POSE_MAX_AGE_S)

    @property
    def rot(self):
        return self.pose_tracker.rot

    @property
    def odom(self):
        return self.pose_tracker.odom

    @property
    def state(self):
        return self.pose_tracker.state


TASK_EXECUTOR = TaskExecutor(
    on_error=lambda error: nav_error(f"task failed: {error}")
)
V1_STOP_EVENT = threading.Event()
# Lock order is command lock -> executor lock. Robot tasks never execute under
# the executor lock. This serializes admission/stop and command publication.
COMMAND_LOCK = threading.Lock()
TASK_CONTEXT = threading.local()


class MotionTimeout(TimeoutError):
    """The movement or task exceeded its monotonic deadline."""


class PoseUnavailable(RuntimeError):
    """A movement cannot continue without a recent valid pose."""


def _check_task():
    if V1_STOP_EVENT.is_set():
        raise TaskCancelled('Task cancelled by stop request')
    if time.monotonic() >= getattr(TASK_CONTEXT, 'deadline', math.inf):
        raise MotionTimeout('Task execution timed out')


def _call_action(method, *args):
    # Prevent a checked-but-not-yet-published movement from overtaking a stop.
    with COMMAND_LOCK:
        _check_task()
        return method(*args)


def _task_wait(interval):
    _check_task()
    remaining = getattr(TASK_CONTEXT, 'deadline', math.inf) - time.monotonic()
    V1_STOP_EVENT.wait(max(0.0, min(interval, remaining)))
    _check_task()


def do_delay(method, interval):
    """Preserve action delays while allowing a stop to abort the whole route."""
    def run():
        _call_action(method)
        _task_wait(interval)
    return run


def _motion_deadline(deadline=None):
    return min(time.monotonic() + MOTION_TIMEOUT_S,
               getattr(TASK_CONTEXT, 'deadline', math.inf),
               deadline if deadline is not None else math.inf)


def _check_motion(deadline):
    _check_task()
    if time.monotonic() >= deadline:
        raise MotionTimeout('Movement timed out')
    if not node.pose_tracker.is_fresh(POSE_MAX_AGE_S):
        raise PoseUnavailable('Robot pose is missing, invalid, or stale')


def _motion_wait(interval, deadline):
    V1_STOP_EVENT.wait(max(0.0, min(interval, deadline - time.monotonic())))
    _check_motion(deadline)


def _run_task(task):
    TASK_CONTEXT.deadline = time.monotonic() + TASK_TIMEOUT_S
    try:
        _check_task()
        task()
        _check_task()
    except Exception:
        # Still busy here: no subsequent task can start before the stop.
        with COMMAND_LOCK:
            node.spc.stop_move()
        raise
    finally:
        del TASK_CONTEXT.deadline


def _submit_task(task):
    with COMMAND_LOCK:
        return TASK_EXECUTOR.submit_handle(
            lambda: _run_task(task), on_accept=V1_STOP_EVENT.clear)


def _stop_tasks(handle=None, error=None):
    with COMMAND_LOCK:
        cleared = TASK_EXECUTOR.cancel(
            handle, on_cancel=V1_STOP_EVENT.set, error=error)
        # A late timeout for an already-finished task must not stop its successor.
        if cleared is not None:
            node.spc.stop_move()
        return cleared


def move(x, y):
    global node
    spc = node.spc
    oac = node.oac

    # rectify = math.pi/36
    rectify = math.pi/8
    delta = node.rot[2] - start_rot[2]
    
    max_rectify = math.pi/2

    # 处理角度跨越±π的情况
    while delta > math.pi:
        delta -= 2 * math.pi
    while delta < -math.pi:
        delta += 2 * math.pi

    # gain = 1.4  # 增益系数，可调整
    gain = 1.8  # 增益系数，可调整
    
    # 计算角度修正量，使用比例控制
    angular_correction = 0
    if abs(delta) > rectify / 36:
        angular_correction = -delta * gain 
        # 限制最大修正量
        angular_correction = max(-max_rectify, min(max_rectify, angular_correction))
    
    res = _call_action(spc.move, x, y, angular_correction)
    # node.get_logger().info(str(x)+","+str(y)+","+str(res))
    # print(delta*180)


def change_yaw(angle, vyaw, deadline=None):
    deadline = _motion_deadline(deadline)
    _check_motion(deadline)
    # node.rot[2]为偏航角
    actual_vyaw = vyaw
    start_yaw = node.rot[2]
    # node.get_logger().info("start_yaw"+str(start_yaw))
    current_yaw = start_yaw
    while True:
        _check_motion(deadline)
        changed_yaw_each_step = node.rot[2] - current_yaw
        if changed_yaw_each_step > math.pi:
            current_yaw = node.rot[2] - math.pi * 2
        elif changed_yaw_each_step < -math.pi:
            current_yaw = node.rot[2] + math.pi * 2
        else:
            current_yaw = node.rot[2]
        rotated_angle = abs(current_yaw - start_yaw)
        # node.get_logger().info("current_yaw"+str(current_yaw/math.pi)+"pi")
        delta_angle = abs(rotated_angle - angle)
        # node.get_logger().info("rotated_angle"+str(rotated_angle/math.pi)+"pi")
        if delta_angle < 0.03 * math.pi:
            actual_vyaw = vyaw * (delta_angle / 0.03 / math.pi)
            if actual_vyaw > 0:
                actual_vyaw = max(math.pi / 16, actual_vyaw)
            else:
                actual_vyaw = min(-math.pi / 16, actual_vyaw)
        if rotated_angle >= angle:
            node.spc.stop_move()
            break
        # node.get_logger().info("actual_vyaw"+str(actual_vyaw/math.pi)+"pi")
        _call_action(node.spc.move, 0, 0, actual_vyaw)
        _motion_wait(0.05, deadline)

def gx(dis, v, deadline=None):
    deadline = _motion_deadline(deadline)
    _check_motion(deadline)
    actual_v = v
    start_pos = node.odom.copy()
    global start_rot
    global flag
    start_rot = node.rot
    while True:
        _check_motion(deadline)
        current_pos = node.odom
        nav_info(f"gx current_pos={current_pos}, actual_v={actual_v}")
        if v<0:
            actual_v = -0.3
        move(actual_v, 0)
        _motion_wait(0.2, deadline)
        traveled_distance = np.linalg.norm(current_pos[:2] - start_pos[:2])
        delta = abs(traveled_distance - dis)
        nav_info(f"gx delta={delta}")
        if delta < 0.3:
            actual_v = v * (delta / 0.2)
            if actual_v > 0:
                actual_v = max(0.2, actual_v)
            else:
                actual_v = min(-0.2, actual_v)
        if traveled_distance >= dis:
            nav_info("gx end move")
            break

def gx_without_refresh_start(dis, v, deadline=None):
    deadline = _motion_deadline(deadline)
    _check_motion(deadline)
    actual_v = v
    start_pos = node.odom.copy()
    while True:
        _check_motion(deadline)
        current_pos = node.odom
        nav_info(f"gx_without_refresh_start current_pos={current_pos}, actual_v={actual_v}")
        if v<0:
            actual_v = -0.3
        move(actual_v, 0)
        _motion_wait(0.2, deadline)
        traveled_distance = np.linalg.norm(current_pos[:2] - start_pos[:2])
        delta = abs(traveled_distance - dis)
        nav_info(f"gx_without_refresh_start delta={delta}")
        if delta < 0.3:
            actual_v = v * (delta / 0.2)
            if actual_v > 0:
                actual_v = max(0.2, actual_v)
            else:
                actual_v = min(-0.2, actual_v)
        if traveled_distance >= dis:
            nav_info("gx_without_refresh_start end move")
            break

def gy(dis, v, deadline=None):
    deadline = _motion_deadline(deadline)
    _check_motion(deadline)
    actual_v = v
    start_pos = node.odom.copy()
    global start_rot
    start_rot = node.rot
    while True:
        _check_motion(deadline)
        current_pos = node.odom
        nav_info(f"gy current_pos={current_pos}, actual_v={actual_v}")
        move(0, actual_v)
        _motion_wait(0.2, deadline)
        traveled_distance = np.linalg.norm(current_pos[:2] - start_pos[:2])
        delta = abs(traveled_distance - dis)
        nav_info(f"gy delta={delta}")
        if delta < 0.3:
            actual_v = v * (delta / 0.2)
            if actual_v > 0:
                actual_v = max(0.2, actual_v)
            else:
                actual_v = min(-0.2, actual_v)
        if traveled_distance >= dis:
            nav_info("gy end move")
            break

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def turn_to_yaw(target_yaw, vyaw, tolerance=math.pi / 72, deadline=None):
    deadline = _motion_deadline(deadline)
    _check_motion(deadline)
    yaw_delta = normalize_angle(target_yaw - node.rot[2])
    nav_info(f"turn_to_yaw delta yaw={yaw_delta}")
    if abs(yaw_delta) <= tolerance:
        return

    change_yaw(abs(yaw_delta), math.copysign(abs(vyaw), yaw_delta),
               deadline=deadline)
    node.spc.stop_move()

def move_to_pos(target_pos, v, tolerance=0.1, timeout_s=20):
    target_xy = np.array(target_pos[:2], dtype=np.float64)
    deadline = _motion_deadline(time.monotonic() + timeout_s)

    while True:
        _check_motion(deadline)
        current_pos = node.odom.copy()
        delta = target_xy - current_pos[:2]
        distance = np.linalg.norm(delta)
        nav_info(f"back original delta={delta}, delta distance={distance}")

        if distance <= tolerance:
            nav_info("already arrived original_pos")
            break

        target_yaw = math.atan2(delta[1], delta[0])
        turn_to_yaw(target_yaw, math.pi / 8 * 3, deadline=deadline)

        walk_distance = min(max(distance - tolerance, 0.0), 0.8)
        if walk_distance <= 0:
            break

        gx(walk_distance, v, deadline=deadline)
        do_delay(node.spc.stop_move, 0.2)()

    node.spc.stop_move()


class RequestHandler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(10.0)

    def log_message(self, format, *args):
        nav_info("http " + (format % args))

    def do_GET(self):
        if urlsplit(self.path).path in {"/v1/status", "/api/v1/status"}:
            send_json(self, 200, {
                "ok": True,
                "busy": TASK_EXECUTOR.busy,
                "pose_ready": node.pose_ready,
                "original_position_recorded": original_pos is not None,
            })
            return
        send_method_not_allowed(self)

    def do_POST(self):
        body, body_text = read_request_body(self)
        nav_info(f"post received path={self.path} body={body_text}")
        path = urlsplit(self.path).path

        if path.startswith('/v1/'):
            self._handle_v1(path, body, body_text)
            return

        if path == '/mock-move':
            self._mock_move()

        # 移动样机
        elif path == '/mock-move-mobile':
            self._mock_move_mobile()
        elif path == '/mock-back-mobile':
            self._mock_back_mobile()
        elif path == '/mock-back-original':
            self._mock_back_original()

        # 架构样机
        elif path == '/mock-move1':
            self._mock_move1()
        elif path == '/mock-move2':
            self._mock_move2()
        elif path == '/mock-move3':
            self._mock_move2()
        elif path == '/mock-move4':
            self._mock_move4()
        elif path == '/mock-back':
            self._mock_back()

        elif path == '/goleft':
            self._go_left()
        elif path == '/goright':
            self._go_right()
        elif path == '/coarse-shift':
            self._coarse_shift(body)

        else:
            send_method_not_allowed(self)

    def _handle_v1(self, path, body, body_text):
        """Handle the documented action API on the existing HTTP action server port."""
        try:
            payload = parse_json_body(body)
        except (TypeError, ValueError, json.JSONDecodeError):
            send_error(self, "Invalid JSON body", 400)
            return

        if not isinstance(payload, dict):
            send_error(self, "JSON body must be an object", 400)
            return
        if not validate_timestamp(self, path, body_text, {}, logger=nav_info):
            return

        request_id = payload.get("request_id") or uuid.uuid4().hex
        if not isinstance(request_id, str) or not request_id.strip():
            send_error(self, "request_id must be a non-empty string", 400)
            return

        if path in {"/v1/stop", "/v1/stop-move"}:
            cleared = _stop_tasks()
            send_json(self, 200, {
                "ok": True,
                "command_id": request_id,
                "state": "stopped",
                "cleared": cleared,
            })
            return

        task = self._build_v1_task(path, payload)
        if task is None:
            return

        if _submit_task(task) is None:
            send_json(self, 409, {
                "ok": False,
                "command_id": request_id,
                "state": "busy",
            })
            return

        send_json(self, 202, {
            "ok": True,
            "command_id": request_id,
            "state": "queued",
        })

    def _build_v1_task(self, path, payload):
        if path == "/v1/move":
            vx = self._number(payload, "vx", -2.5, 3.8)
            vy = self._number(payload, "vy", -1.0, 1.0)
            vyaw = self._number(payload, "vyaw", -4.0, 4.0)
            if vx is None or vy is None or vyaw is None:
                return None
            return lambda: _call_action(node.spc.move, vx, vy, vyaw)

        if path == "/v1/euler":
            roll = self._number(payload, "roll", -0.75, 0.75)
            pitch = self._number(payload, "pitch", -0.75, 0.75)
            yaw = self._number(payload, "yaw", -0.6, 0.6)
            if roll is None or pitch is None or yaw is None:
                return None
            method = getattr(node.spc, "euler", None)
            if not callable(method):
                send_error(self, "SportClient.euler is not supported", 501)
                return None
            return lambda: _call_action(method, roll, pitch, yaw)

        if path == "/v1/speed-level":
            try:
                level = int(payload["level"])
            except (KeyError, TypeError, ValueError):
                send_error(self, "level must be -1, 0, or 1", 400)
                return None
            if level not in {-1, 0, 1}:
                send_error(self, "level must be -1, 0, or 1", 400)
                return None
            method = getattr(node.spc, "speed_level", None)
            if not callable(method):
                send_error(self, "SportClient.speed_level is not supported", 501)
                return None
            return lambda: _call_action(method, level)

        if path in {"/v1/forward", "/v1/backward", "/v1/left", "/v1/right"}:
            distance = self._number(payload, "distance_m", 0.05, 2.0)
            speed = self._number(payload, "speed_mps", 0.1, 0.5)
            if distance is None or speed is None:
                return None
            direction = {
                "/v1/forward": (1, 0),
                "/v1/backward": (-1, 0),
                "/v1/left": (0, 1),
                "/v1/right": (0, -1),
            }[path]
            return lambda: (
                gx(distance, direction[0] * speed)
                if direction[0]
                else gy(distance, direction[1] * speed)
            )

        if path in {"/v1/turn-back", "/v1/turn-left", "/v1/turn-right"}:
            angle = self._number(payload, "angle_deg", 1.0, 360.0)
            speed = self._number(payload, "speed_degps", 10.0, 180.0)
            if angle is None or speed is None:
                return None
            sign = {"/v1/turn-back": 1, "/v1/turn-left": 1,
                    "/v1/turn-right": -1}[path]
            return lambda: change_yaw(
                math.radians(angle), sign * math.radians(speed)
            )

        actions = {
            "/v1/wave": "hello",
            "/v1/handshake": "handshake",
            "/v1/pounce": "front_pounce",
            "/v1/front-pounce": "front_pounce",
            "/v1/front-jump": "front_jump",
            "/v1/scrape": "scrape",
            "/v1/front-flip": "front_flip",
            "/v1/hand-stand": "hand_stand",
            "/v1/left-flip": "left_flip",
            "/v1/back-flip": "back_flip",
            "/v1/stretch": "stretch",
            "/v1/happy": "content",
            "/v1/content": "content",
            "/v1/hello": "hello",
            "/v1/stand-up": "stand_up",
            "/v1/stand-down": "stand_down",
            "/v1/balance-stand": "balanced_stand",
            "/v1/recovery-stand": "recovery_stand",
            "/v1/sit": "sit",
            "/v1/rise-sit": "rise_sit",
            "/v1/heart": "heart",
            "/v1/dance-1": "dance_1",
            "/v1/dance-2": "dance_2",
        }
        method_name = actions.get(path)
        if method_name is None:
            send_error(self, "Unknown v1 action", 404)
            return None
        method = getattr(node.spc, method_name, None)
        if not callable(method):
            send_error(self, f"SportClient.{method_name} is not supported", 501)
            return None
        return lambda: _call_action(method)

    def _number(self, payload, name, minimum, maximum):
        try:
            value = float(payload[name])
        except (KeyError, TypeError, ValueError):
            send_error(self, f"{name} must be a number", 400)
            return None
        if not math.isfinite(value) or not minimum <= value <= maximum:
            send_error(self, f"{name} must be between {minimum} and {maximum}", 400)
            return None
        return value

    def _response_error(self, msg):
        send_error(self, msg)

    def _enqueue_task(self, task):
        if _submit_task(task) is None:
            self._response_error(
                'Busy: previous movement task is still running, please wait until it finishes'
            )
            return
        self._respond_ok()

    def _enqueue_task_and_wait(self, task, ok_body=b'OK'):
        handle = _submit_task(task)
        if handle is None:
            self._response_error(
                'Busy: previous movement task is still running, please wait until it finishes'
            )
            return

        nav_info("task enqueued, waiting for completion before HTTP response")
        if not handle.done.wait(SYNC_WAIT_TIMEOUT_S):
            _stop_tasks(handle, MotionTimeout('Timed out waiting for movement'))

        if handle.error is None:
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain')
            self.send_header('Content-Length', str(len(ok_body)))
            self.end_headers()
            self.wfile.write(ok_body)
            self.wfile.flush()
            return

        error = handle.error
        nav_error(f"task failed before HTTP response: {error}")
        status = (409 if isinstance(error, TaskCancelled) else
                  504 if isinstance(error, TimeoutError) else
                  503 if isinstance(error, PoseUnavailable) else 500)
        send_text(self, status, f'Task failed: {error}')

    def _mock_move(self):
        global node
        nav_info("mock_move requested")
        def mock_move_task():
            spc = node.spc
            oac = node.oac
            vuc = node.vuc
            v = 0.4
            v_yaw = math.pi / 8 *3
            #res=spc.euler(-node.rot[0],-node.rot[1],0)
            if node.state in ['damping', 'lieDown']:
                do_delay(spc.stand_up, 0.4)()
                do_delay(spc.balanced_stand, 0.1)()
                do_delay(oac.disable, 0.05)()

            # 按照指定路线行走
            # 1.前进1.5m； 2.左转90度； 3.右挪0.2m； 4.坐下

            gx(1.2, v)
            do_delay(spc.stop_move, 0.2)()

            change_yaw(math.pi/2*0.95, v_yaw)
            do_delay(spc.stop_move, 0.2)()
            
            gx(0.05, v) # 防止没站稳跌到
            do_delay(spc.stop_move, 0.2)()
            nav_info("mock_move closer")
            gy(0.15,-v)
            do_delay(spc.stop_move, 0.2)()
            do_delay(spc.balanced_stand, 1)()

        self._enqueue_task(mock_move_task)

    def _mock_move_mobile(self):
        global node
        nav_info("[mobile] mock_move_mobile requested")
        def mock_move_mobile():
            nav_info("[mobile] mock_move_mobile start")
            global original_pos
            spc = node.spc
            oac = node.oac
            vuc = node.vuc
            v = 0.4

            original_pos = node.odom.copy()
            nav_info(f"[mobile] record original_pos={original_pos}")
            gx(0.8, v)
            do_delay(spc.stop_move, 0.2)()
            nav_info("[mobile] mock_move_mobile success")

        self._enqueue_task(mock_move_mobile)


    def _mock_back_mobile(self):
        global node
        nav_info("[mobile] mock_back_mobile requested")
        def mock_back_mobile_task():
            nav_info("[mobile] mock_back_mobile start")
            spc = node.spc
            oac = node.oac
            vuc = node.vuc
            v = 0.4
            v_yaw = math.pi / 8 *3

            change_yaw(math.pi * 0.95, v_yaw)
            do_delay(spc.stop_move, 0.2)()
            gx(0.05, v) # 防止没站稳跌到
            do_delay(spc.stop_move, 0.2)()
            gx(1.1, v)
            do_delay(spc.stop_move, 0.2)()
            nav_info("[mobile] mock_back_mobile success")


        self._enqueue_task(mock_back_mobile_task)

    def _mock_back_original(self):
        global node
        global original_pos

        if original_pos is None:
            self._response_error('original_pos is not recorded yet; call /mock-move-mobile first')
            return

        nav_info("[mobile] mock_back_original requested")

        def mock_back_original_task():
            nav_info("[mobile] mock_back_original start")
            spc = node.spc
            target_pos = original_pos.copy()
            nav_info(f"[mobile] back to original_pos={target_pos}")
            move_to_pos(target_pos, 0.4)
            do_delay(spc.stop_move, 0.2)()
            nav_info("[mobile] mock_back_original success")

        self._enqueue_task(mock_back_original_task)

    def _mock_move1(self):
        global node
        nav_info("[arch] mock_move1 requested")
        def mock_move_task1():
            nav_info("[arch] mock_move1 start")
            global original_pos
            spc = node.spc
            oac = node.oac
            vuc = node.vuc
            v = 0.4
            original_pos = node.odom.copy()
            nav_info(f"[arch] record original_pos={original_pos}")

            gx(0.3, v)
            do_delay(spc.stop_move, 0.2)()
            nav_info("[arch] mock_move1 success")

        self._enqueue_task(mock_move_task1)
    
    def _mock_move2(self):
        global node
        nav_info("[arch] mock_move2 requested")
        def mock_move_task2():
            nav_info("[arch] mock_move2 start")
            spc = node.spc
            oac = node.oac
            vuc = node.vuc
            v = 0.4
            turn_to_yaw(start_rot[2], math.pi / 8 * 3)
            do_delay(spc.stop_move, 0.2)()
            gx_without_refresh_start(0.3, v)
            do_delay(spc.stop_move, 0.2)()
            nav_info("[arch] mock_move2 success")

        self._enqueue_task(mock_move_task2)

    def _mock_move4(self):
        global node
        nav_info("[arch] mock_move4 requested")
        def mock_move_task4():
            nav_info("[arch] mock_move4 start")
            spc = node.spc
            oac = node.oac
            vuc = node.vuc
            v = 0.4
            v_yaw = math.pi / 8 *3
            turn_to_yaw(start_rot[2], math.pi / 8 * 3)
            do_delay(spc.stop_move, 0.2)()
            gx(0.3, v)
            do_delay(spc.stop_move, 0.2)()
            change_yaw(math.pi/2*0.95, v_yaw)
            do_delay(spc.stop_move, 0.2)()
            
            gx(0.3, v) # 防止没站稳跌到
            do_delay(spc.stop_move, 0.2)()
            nav_info("[arch] mock_move4 closer")
            gy(0.2,-v)
            do_delay(spc.stop_move, 0.2)()
            do_delay(spc.balanced_stand, 1)()
            nav_info("[arch] mock_move4 success")

        self._enqueue_task(mock_move_task4)


    def _mock_back(self):
        global node
        nav_info("[arch] mock_back requested")
        def mock_back_task():
            nav_info("[arch] mock_back start")
            spc = node.spc
            oac = node.oac
            vuc = node.vuc
            v = 0.4
            v_yaw = math.pi / 8 *3

            # 按照指定路线行走
            # 1.左挪0.4m； 2.左转90度； 3.前进2m； 4.左转180度；  5. 坐下
            gy(0.25, v)
            do_delay(spc.stop_move, 0.2)()
            change_yaw(math.pi/2*0.95, v_yaw)
            do_delay(spc.stop_move, 0.2)()
            gx(0.05, v) # 防止没站稳跌到
            do_delay(spc.stop_move, 0.2)()
            gx(1.1, v)
            do_delay(spc.stop_move, 0.2)()
            change_yaw(math.pi * 0.95, v_yaw)
            do_delay(spc.stop_move, 0.2)()
            gx(0.05, v) # 防止没站稳跌到
            do_delay(spc.stop_move, 0.2)()
            nav_info("[arch] mock_back success")
           
        self._enqueue_task(mock_back_task)


    def _go_left(self):
        global node
        nav_info("go_left requested")
        def go_left_task():
            spc = node.spc
            v = 0.4
            nav_info(f"go_left rot={node.rot}")
            gy(0.2, v)
            do_delay(spc.stop_move, 0.2)()

            # node.get_logger().info("The robot dog has arrived at the merchant")

        self._enqueue_task(go_left_task)

    def _go_right(self):
        global node

        nav_info("go_right requested")
        def go_right_task():
            spc = node.spc
            v = 0.4
            nav_info(f"go_right rot={node.rot}")
            gy(0.2, -v)
            do_delay(spc.stop_move, 0.2)()

            # node.get_logger().info("The robot dog has arrived at the merchant")

        self._enqueue_task(go_right_task)

    def _coarse_shift(self, body):
        global node

        try:
            payload = parse_json_body(body)
            distance = float(payload.get('distance'))
            if not math.isfinite(distance):
                raise ValueError('distance must be finite')
        except Exception:
            self._response_error('Invalid distance')
            return

        nav_info(f"[mobile] coarse_shift distance={distance}")
        if distance == 0:
            nav_info("[mobile] coarse_shift distance=0, skip movement")
            self._respond_ok()
            return

        def coarse_shift_task():
            nav_info("[mobile] coarse_shift start")
            spc = node.spc
            v = 0.4
            gy(abs(distance), -v if distance > 0 else v)
            do_delay(spc.stop_move, 0.2)()
            nav_info("[mobile] coarse_shift success")

        self._enqueue_task_and_wait(coarse_shift_task)



    def _respond_ok(self):
        send_ok(self)

def main(args=None):
    rclpy.init(args=args)
    global node
    node = HttpActionServer()
    srv = ThreadingHTTPServer(('0.0.0.0', HTTP_ACTION_PORT), RequestHandler)
    nav_info('HTTP server started: http://0.0.0.0:10088')
    try:
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
        spin_thread.start()
        TASK_EXECUTOR.start()
        nav_info('starting server, use <Ctrl-C> to stop')
        srv.serve_forever()

    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        _stop_tasks()
        TASK_EXECUTOR.stop()
        srv.server_close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
