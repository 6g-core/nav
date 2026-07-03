import rclpy
from rclpy.node import Node
import time
import math
import os
import threading
import json
import http.client
from datetime import datetime
from urllib.parse import parse_qs, urlparse

from clients.obstacles_avoid_client import ObstaclesAvoidClient
from clients.sport_client import SportClient
from clients.utils import *
from http.server import BaseHTTPRequestHandler, HTTPServer
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
import numpy as np
from clients.vui_client import VUIClient
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
node = None
start_rot = None
original_pos = None

MOBILE_APP_IP = "192.168.1.101"
MOBILE_APP_PORT = 8000
MOBILE_APP_DEST = "/dogfinish"

ARM_PLANB_IP = "192.168.1.106"
ARM_PLANB_PORT = 10088
ARM_PLANB_DEST = "/move"

DOG_PLANB_PORT = 10088
REQUEST_TIMESTAMP_TIMEOUT_S = 20.0


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


class NavStubbingPlanB(Node):
    def __init__(self):
        super().__init__('nav_stubbing_planB_node')
        self.state = "unknown"

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

        self.rot = np.array([0, 0, 0])
        self.odom = np.array([ 0, 0, 0 ])
        self.scan = None
        self.state = None

    def handle_state(self, msg: String):
        self.state = msg.data
    
    def handle_scan(self, msg: LaserScan):
        self.scan = msg

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert quaternion to euler angles (roll, pitch, yaw)
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw], dtype=np.float64)

    def handle_pose(self, msg: PoseStamped):
        quat = np.zeros(4)
        quat[0] = msg.pose.orientation.x
        quat[1] = msg.pose.orientation.y
        quat[2] = msg.pose.orientation.z
        quat[3] = msg.pose.orientation.w
        euler = self.quaternion_to_euler(*quat)
        self.rot = euler

        self.odom = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

TASK_QUEUE = []
TASK_LOCK = threading.Lock()
TASK_RUNNING = False


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

    # gain = 1.8  # 增益系数，可调整
    gain = 2.0
    
    # 计算角度修正量，使用比例控制
    angular_correction = 0
    if abs(delta) > rectify / 36:
        angular_correction = -delta * gain 
        # 限制最大修正量
        angular_correction = max(-max_rectify, min(max_rectify, angular_correction))
    
    res=spc.move(x, y, angular_correction)
    # node.get_logger().info(str(x)+","+str(y)+","+str(res))
    # print(delta*180)


def change_yaw(angle, vyaw):
    # node.rot[2]为偏航角
    actual_vyaw = vyaw
    start_yaw = node.rot[2]
    # node.get_logger().info("start_yaw"+str(start_yaw))
    current_yaw = start_yaw
    while True:
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
                actual_vyaw = max(math.pi / 8, actual_vyaw)
            else:
                actual_vyaw = min(-math.pi / 8, actual_vyaw)
        if rotated_angle >= angle:
            node.spc.stop_move()
            break
        # node.get_logger().info("actual_vyaw"+str(actual_vyaw/math.pi)+"pi")
        node.spc.move(0, 0, actual_vyaw)
        time.sleep(0.05)

def gx(dis, v):
    actual_v = v
    start_pos = node.odom.copy()
    global start_rot
    global flag
    start_rot = node.rot
    while True:
        current_pos = node.odom
        nav_info(f"gx current_pos={current_pos}, actual_v={actual_v}")
        if v<0:
            actual_v = -0.3
        move(actual_v, 0)
        time.sleep(0.2)
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

def gx_without_refresh_start(dis, v):
    actual_v = v
    start_pos = node.odom.copy()
    while True:
        current_pos = node.odom
        nav_info(f"gx_without_refresh_start current_pos={current_pos}, actual_v={actual_v}")
        if v<0:
            actual_v = -0.3
        move(actual_v, 0)
        time.sleep(0.2)
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

def gy(dis, v):
    actual_v = v
    start_pos = node.odom.copy()
    global start_rot
    start_rot = node.rot
    while True:
        current_pos = node.odom
        nav_info(f"gy current_pos={current_pos}, actual_v={actual_v}")
        move(0, actual_v)
        time.sleep(0.2)
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

def turn_to_yaw(target_yaw, vyaw, tolerance=math.pi / 72):
    yaw_delta = normalize_angle(target_yaw - node.rot[2])
    nav_info(f"turn_to_yaw delta yaw={yaw_delta}")
    if abs(yaw_delta) <= tolerance:
        return

    change_yaw(abs(yaw_delta), math.copysign(abs(vyaw), yaw_delta))
    node.spc.stop_move()

def move_to_pos(target_pos, v, tolerance=0.1, timeout_s=20):
    target_xy = np.array(target_pos[:2], dtype=np.float64)
    start_time = time.time()

    while True:
        current_pos = node.odom.copy()
        delta = target_xy - current_pos[:2]
        distance = np.linalg.norm(delta)
        nav_info(f"back original delta={delta}, delta distance={distance}")

        if distance <= tolerance:
            nav_info("already arrived original_pos")
            break

        if time.time() - start_time > timeout_s:
            nav_error("move_to_pos timeout before reaching original_pos")
            break

        target_yaw = math.atan2(delta[1], delta[0])
        turn_to_yaw(target_yaw, math.pi / 8 * 3)

        walk_distance = min(max(distance - tolerance, 0.0), 0.8)
        if walk_distance <= 0:
            break

        gx(walk_distance, v)
        do_delay(node.spc.stop_move, 0.2)()

    node.spc.stop_move()


class RequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        nav_info("http " + (format % args))

    def do_POST(self):
        content_length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(content_length) if content_length > 0 else b''
        body_text = body.decode('utf-8', errors='replace') if body else ''
        nav_info(f"post received path={self.path} content_length={content_length} body={body_text}")
        parsed_url = urlparse(self.path)
        request_path = parsed_url.path

        timestamp_required_paths = {
            '/mock-move-mobile',
            '/mock-back-original',
            '/mock-move1',
            '/mock-move2',
            '/mock-move3',
            '/mock-move4',
            '/mock-back',
        }
        if request_path in timestamp_required_paths:
            query_params = parse_qs(parsed_url.query)
            if not self._validate_request_timestamp(self.path, body_text, query_params):
                return

        if request_path == '/mock-move':
            self._mock_move()

        # 移动样机
        elif request_path == '/mock-move-mobile':
            self._mock_move_mobile()
        elif request_path == '/mock-back-mobile':
            self._mock_back_mobile()
        elif request_path == '/mock-back-original':
            self._mock_back_original()

        # 架构样机
        elif request_path == '/mock-move1':
            self._mock_move1()
        elif request_path == '/mock-move2':
            self._mock_move2()
        elif request_path == '/mock-move3':
            self._mock_move2()
        elif request_path == '/mock-move4':
            self._mock_move4()
        elif request_path == '/mock-back':
            self._mock_back()

        elif request_path == '/goleft':
            self._go_left()
        elif request_path == '/goright':
            self._go_right()
        elif request_path == '/coarse-shift':
            self._coarse_shift(body)

        else:
            self.send_error(405)
            self.send_header('Content-Type','text/plain')
            self.end_headers()
            self.wfile.write(b"Method Not Allowed")

    def _response_error(self, msg):
        body = (msg + '\n').encode('utf-8')
        self.send_response(400, msg)
        self.send_header('Content-Type','text/plain')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)
        self.wfile.flush()

    def _response_with_status(self, status, msg):
        body = (msg + '\n').encode('utf-8')
        self.send_response(status, msg)
        self.send_header('Content-Type','text/plain')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)
        self.wfile.flush()

    def _validate_request_timestamp(self, request_url, body_text, query_params):
        timestamp = None
        values = query_params.get('timestamp')
        if values:
            timestamp = values[0]
        elif body_text:
            try:
                payload = json.loads(body_text)
                timestamp = payload.get('timestamp')
            except json.JSONDecodeError:
                self._response_error('Invalid JSON body')
                return False

        if timestamp is None:
            self._response_error('Missing timestamp')
            return False

        try:
            request_time = float(timestamp)
            if request_time > 100000000000:
                request_time = request_time / 1000.0
        except (TypeError, ValueError):
            self._response_error('Invalid timestamp')
            return False

        now = time.time()
        age = abs(now - request_time)
        nav_info(
            f"timestamp check url={request_url} timestamp={timestamp} "
            f"request_time={request_time:.3f} current_time={now:.3f} age={age:.3f}s"
        )
        if age > REQUEST_TIMESTAMP_TIMEOUT_S:
            nav_info(f"request expired url={request_url} timestamp={timestamp} age={age:.3f}s")
            self._response_with_status(408, 'Request timestamp expired')
            return False

        return True

    def _enqueue_task(self, task):
        global TASK_RUNNING

        with TASK_LOCK:
            if TASK_RUNNING or TASK_QUEUE:
                self._response_error('Busy: previous movement task is still running, please wait until it finishes')
                return

            TASK_QUEUE.append(task)
            self._respond_ok()

    def _enqueue_task_and_wait(self, task, ok_body=b'OK'):
        global TASK_RUNNING

        done_event = threading.Event()
        result = {
            "ok": False,
            "error": None,
        }

        def wrapped_task():
            try:
                task()
            except Exception as e:
                result["error"] = e
                raise
            else:
                result["ok"] = True
            finally:
                done_event.set()

        with TASK_LOCK:
            if TASK_RUNNING or TASK_QUEUE:
                self._response_error('Busy: previous movement task is still running, please wait until it finishes')
                return

            TASK_QUEUE.append(wrapped_task)

        nav_info("task enqueued, waiting for completion before HTTP response")
        done_event.wait()

        if result["ok"]:
            self.send_response(200)
            self.send_header('Content-Type','text/plain')
            self.send_header('Content-Length', str(len(ok_body)))
            self.end_headers()
            self.wfile.write(ok_body)
            self.wfile.flush()
            return

        error = result["error"] or "task failed"
        nav_error(f"task failed before HTTP response: {error}")
        body = f"Task failed: {error}\n".encode('utf-8')
        self.send_response(500)
        self.send_header('Content-Type','text/plain')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)
        self.wfile.flush()

    def _run_task_sync(self, task, ok_body=b'OK'):
        global TASK_RUNNING

        with TASK_LOCK:
            if TASK_RUNNING or TASK_QUEUE:
                self._response_error('Busy: previous movement task is still running, please wait until it finishes')
                return

            TASK_RUNNING = True

        try:
            task()
        except Exception as e:
            nav_error(f"task failed: {e}")
            body = f"Task failed: {e}\n".encode('utf-8')
            self.send_response(500)
            self.send_header('Content-Type','text/plain')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            self.wfile.flush()
        else:
            self.send_response(200)
            self.send_header('Content-Type','text/plain')
            self.send_header('Content-Length', str(len(ok_body)))
            self.end_headers()
            self.wfile.write(ok_body)
            self.wfile.flush()
        finally:
            with TASK_LOCK:
                TASK_RUNNING = False

    def post_to(self, url, port, dest, body = {}):
        try:
            conn = http.client.HTTPConnection(url, port, timeout=1)
            conn.request("POST", dest, body=json.dumps(body), headers={"Content-Type": "application/json"})
            response = conn.getresponse()
            nav_info(f"request url={url}:{port}{dest} status={response.status} reason={response.reason}")
            if response.status >= 400:
                nav_error(f"request error status={response.status} reason={response.reason}")
            else:
                nav_info(f"request success status={response.status} reason={response.reason}")
            conn.close()
        except Exception as e:
            nav_error(f"failed to connect url={url}:{port}{dest} error={e}")

    def _mock_move(self):
        global node
        nav_info("mock_move requested")
        def mock_move_task():
            spc = node.spc
            oac = node.oac
            vuc = node.vuc
            v = 0.4
            v_yaw = math.pi / 8 *3
            low_delay = 1.4
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
            nav_info("turn_to_yaw over")
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
            nav_info("turn_to_yaw over")
            do_delay(spc.stop_move, 0.2)()
            gx(0.3, v)
            do_delay(spc.stop_move, 0.2)()
            change_yaw(math.pi/2*0.95, v_yaw)
            do_delay(spc.stop_move, 0.2)()
            
            # gx(0.3, v) # 防止没站稳跌到
            gx(0.2, v)
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
            turn_to_yaw(start_rot[2], math.pi / 8 * 3)
            do_delay(spc.stop_move, 0.2)()
            #gy(0.4,-v)
            #do_delay(spc.stop_move, 0.2)()
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
            payload = json.loads(body.decode('utf-8') if body else '{}')
            distance = float(payload.get('distance'))
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
        self.send_response(200)
        self.send_header('Content-Type','text/plain')
        self.end_headers()
        self.wfile.write(b'OK')
        self.wfile.flush()

def exec_tasks():
    global TASK_RUNNING

    while True:
        task = None
        with TASK_LOCK:
            if TASK_QUEUE:
                task = TASK_QUEUE.pop(0)
                TASK_RUNNING = True

        if task:
            try:
                task()
            except Exception as e:
                nav_error(f"task failed: {e}")
            finally:
                with TASK_LOCK:
                    TASK_RUNNING = False

        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    global node
    node = NavStubbingPlanB()
    srv = HTTPServer(('0.0.0.0', DOG_PLANB_PORT), RequestHandler)
    nav_info('HTTP server started: http://0.0.0.0:10088')
    try:
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
        spin_thread.start()
        task_thread = threading.Thread(target=exec_tasks)
        task_thread.start()
        nav_info('starting server, use <Ctrl-C> to stop')
        srv.serve_forever()

    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
