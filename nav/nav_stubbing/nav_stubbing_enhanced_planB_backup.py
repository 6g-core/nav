import rclpy
from rclpy.node import Node
import time
import math
import os
import threading
import json
import http.client

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

MOBILE_APP_IP = "192.168.1.101"
MOBILE_APP_PORT = 8000
MOBILE_APP_DEST = "/dogfinish"

ARM_PLANB_IP = "192.168.1.106"
ARM_PLANB_PORT = 10088
ARM_PLANB_DEST = "/move"

DOG_PLANB_PORT = 10088

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
                actual_vyaw = max(math.pi / 16, actual_vyaw)
            else:
                actual_vyaw = min(-math.pi / 16, actual_vyaw)
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
    start_rot = node.rot
    while True:
        current_pos = node.odom
        node.get_logger().info(str(current_pos))
        if v<0:
            actual_v = -0.3
        move(actual_v, 0)
        time.sleep(0.2)
        traveled_distance = np.linalg.norm(current_pos[:2] - start_pos[:2])
        delta = abs(traveled_distance - dis)
        node.get_logger().info(str(delta))
        if delta < 0.3:
            actual_v = v * (delta / 0.2)
            if actual_v > 0:
                actual_v = max(0.1, actual_v)
            else:
                actual_v = min(-0.1, actual_v)
            # node.get_logger().info("v"+str(actual_v))
        if traveled_distance >= dis:
            break

def gy(dis, v):
    actual_v = v
    start_pos = node.odom.copy()
    global start_rot
    start_rot = node.rot
    while True:
        current_pos = node.odom
        move(0, actual_v)
        time.sleep(0.2)
        traveled_distance = np.linalg.norm(current_pos[:2] - start_pos[:2])
        delta = abs(traveled_distance - dis)
        node.get_logger().info(str(delta))
        if delta < 0.3:
            actual_v = v * (delta / 0.2)
            if actual_v > 0:
                actual_v = max(0.1, actual_v)
            else:
                actual_v = min(-0.1, actual_v)
            node.get_logger().info("v"+str(actual_v))
        # node.get_logger().info(f"N {traveled_distance}, T {dis}, delta {delta}, v {actual_v}")
        if traveled_distance >= dis:
            break


class RequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/deliver':
            self._deliver_meal()
        elif self.path == '/dogfinish':
            self._finish_deliver()
        else:
            self.send_error(405)
            self.send_header('Content-Type','text/plain')
            self.end_headers()
            self.wfile.write(b"Method Not Allowed")

    def _response_error(self, msg):
        self.send_error(400)
        self.send_header('Content-Type','text/plain')
        self.end_headers()
        self.wfile.write(msg.encode('utf-8'))

    def post_to(self, url, port, dest, body = {}):
        try:
            conn = http.client.HTTPConnection(url, port, timeout=1)
            conn.request("POST", dest, body=json.dumps(body), headers={"Content-Type": "application/json"})
            response = conn.getresponse()
            node.get_logger().info(f"Request to localhost:{port}, status: {response.status}, reason: {response.reason}")
            if response.status >= 400:
                node.get_logger().error(f"\033[91mError: {response.status} {response.reason}\033[0m")
            else:
                node.get_logger().info(f"\033[92mSuccess: {response.status} {response.reason}\033[0m")
            conn.close()
        except Exception as e:
            node.get_logger().error(f"Failed to connect to localhost:{port}: {e}")

    def _deliver_meal(self):
        global node
        node.get_logger().info("Received meal delivery message")
        node.get_logger().info(str(node.rot))
        def fetch_task():
            spc = node.spc
            oac = node.oac
            vuc = node.vuc
            v = 0.4
            v_yaw = math.pi / 8 *3
            low_delay = 1.4
            #res=spc.euler(-node.rot[0],-node.rot[1],0)
            #node.get_logger().info(str(res))
            node.get_logger().info(str(node.rot))
            if node.state in ['damping', 'lieDown']:
                do_delay(spc.stand_up, 0.4)()
                do_delay(spc.balanced_stand, 0.1)()
                do_delay(oac.disable, 0.05)()

            # 按照指定路线行走
            # 1.前进1m； 2.左转90度； 3.前进2m； 4.坐下
            # gx(1.25, v)
            gx(0.8, v)
            do_delay(spc.stop_move, 0.2)()
            change_yaw(math.pi/2*0.95, v_yaw)
            do_delay(spc.stop_move, 0.2)()
            gx(0.05, v) # 防止没站稳跌到
            do_delay(spc.stop_move, 0.2)()
            gx(1.65,v)
            do_delay(spc.stop_move, 0.2)()
            do_delay(spc.balanced_stand, 1)()
            do_delay(spc.stand_up, 0.5)()
            do_delay(spc.stand_down, 0.5)()
            node.get_logger().info("The robot dog has arrived at the merchant")

        with TASK_LOCK:
            if len(TASK_QUEUE) == 0:
                TASK_QUEUE.append(fetch_task)
                self._respond_ok()
            else:
                self._response_error('Busy')
    
    def _finish_deliver(self):
        global node
        node.get_logger().info("Received finish delivery message")
        def deliver_task():
            spc = node.spc
            oac = node.oac
            vuc = node.vuc
            v = 0.4
            v_yaw = math.pi / 8 *3
            low_delay = 1.4
            # 1.平衡站立； 2.旋转180度； 3.前进2m； 4.右转90度； 5.前进1m； 6.坐下
            do_delay(spc.stand_up, 0.5)()
            time.sleep(2)
            do_delay(spc.balanced_stand, 1)()            

            time.sleep(0.5)
            change_yaw(math.pi*0.97, v_yaw)
            do_delay(spc.stop_move, 0.2)()
            gx(1.65,v)
            do_delay(spc.stop_move, 0.2)()
            change_yaw(math.pi/2*1.03, -v_yaw)
            do_delay(spc.stop_move, 0.2)()
            gx(0.8,v)
            do_delay(spc.stop_move, 0.2)()
            do_delay(spc.balanced_stand, 1)()
            do_delay(spc.stand_up, 0.5)()
            do_delay(spc.stand_down, 0.5)()
            node.get_logger().info("The robot dog has arrived at home")

        with TASK_LOCK:
            if len(TASK_QUEUE) == 0:
                TASK_QUEUE.append(deliver_task)
                self._respond_ok()
            else:
                self._response_error('Busy')

    def _respond_ok(self):
        self.send_response(200)
        self.send_header('Content-Type','text/plain')
        self.end_headers()
        self.wfile.write(b'OK')
        self.wfile.flush()

def exec_tasks():
    while True:
        with TASK_LOCK:
            if TASK_QUEUE:
                task = TASK_QUEUE.pop(0)
                task()
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    global node
    node = NavStubbingPlanB()
    srv = HTTPServer(('0.0.0.0', DOG_PLANB_PORT), RequestHandler)
    node.get_logger().info('HTTP服务器启动：http://0.0.0.0:10088')
    try:
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
        spin_thread.start()
        task_thread = threading.Thread(target=exec_tasks)
        task_thread.start()
        print('Starting server, use <Ctrl-C> to stop')
        srv.serve_forever()

    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
