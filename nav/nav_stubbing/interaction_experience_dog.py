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
from threading import Event

node = None # ROS节点全局变量
start_rot = np.array([0, 0, 0])

DOG_INTERACT_PORT = 8080

IS_RUNNING = False  # 标记当前是否有任务在执行
TASK_QUEUE = []
TASK_LOCK = threading.Lock()


class InteractionExperience(Node):
    def __init__(self):
        super().__init__('robot_dog_interaction_experience')
                
        self.state = None
        self.rot = np.array([0, 0, 0])
        self.odom = np.array([ 0, 0, 0 ])

        self.spc = SportClient(self)
        self.oac = ObstaclesAvoidClient(self)
        self.vuc = VUIClient(self)
    
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
        self.state = msg.data

    def quaternion_to_euler(self, x, y, z, w):
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


def move(x, y):
    global node
    spc = node.spc
    oac = node.oac

    global start_rot

    rectify = math.pi/8
    delta = node.rot[2] - start_rot[2]
    
    max_rectify = math.pi/2

    # 处理角度跨越±π的情况
    while delta > math.pi:
        delta -= 2 * math.pi
    while delta < -math.pi:
        delta += 2 * math.pi

    gain = 1.8  # 增益系数，可调整
    
    # 计算角度修正量，使用比例控制
    angular_correction = 0
    if abs(delta) > rectify / 36:
        angular_correction = -delta * gain 
        # 限制最大修正量
        angular_correction = max(-max_rectify, min(max_rectify, angular_correction))
    
    res=spc.move(x, y, angular_correction)
    node.get_logger().info(str(x)+","+str(y)+","+str(res))
    # print(delta*180)


def change_yaw(angle, vyaw):
    # node.rot[2]为偏航角
    actual_vyaw = vyaw
    start_yaw = node.rot[2]
    node.get_logger().info("start_yaw"+str(start_yaw))
    current_yaw = start_yaw
    while True:
        changed_yaw_each_step = node.rot[2] - current_yaw
        if changed_yaw_each_step > math.pi:
            current_yaw = node.rot[2] - math.pi * 2
        elif changed_yaw_each_step < -math.pi:
            current_yaw = node.rot[2] + math.pi * 2
        else:
            current_yaw = node.rot[2]
        #node.get_logger().info("current_yaw"+str(current_yaw))
        rotated_angle = abs(current_yaw - start_yaw)
        node.get_logger().info("current_yaw"+str(current_yaw/math.pi)+"pi")
        delta_angle = abs(rotated_angle - angle)
        node.get_logger().info("rotated_angle"+str(rotated_angle/math.pi)+"pi")
        if delta_angle < 0.03 * math.pi:
            actual_vyaw = vyaw * (delta_angle / 0.03 / math.pi)
            if actual_vyaw > 0:
                actual_vyaw = max(math.pi / 16, actual_vyaw)
            else:
                actual_vyaw = min(-math.pi / 16, actual_vyaw)
        if rotated_angle >= angle:
            node.spc.stop_move()
            break
        node.get_logger().info("actual_vyaw"+str(actual_vyaw/math.pi)+"pi")
        node.spc.move(0, 0, actual_vyaw)
        time.sleep(0.05)

def gx(dis, v):
    actual_v = v
    start_pos = node.odom.copy()
    global start_rot
    start_rot = node.rot.copy()
    while True:
        current_pos = node.odom
        node.get_logger().info(str(current_pos))
        # if v<0:
        #     actual_v = -0.3
        move(actual_v, 0)
        time.sleep(0.2)
        traveled_distance = np.linalg.norm(current_pos[:2] - start_pos[:2])
        delta = abs(traveled_distance - dis)
        node.get_logger().info(str(delta))
        if delta < 0.3:
            actual_v = v * (delta / 0.2)
            if actual_v > 0:
                actual_v = max(0.2, actual_v)
            else:
                actual_v = min(-0.2, actual_v)
            node.get_logger().info("v"+str(actual_v))
        # node.get_logger().info(f"N {traveled_distance}, T {dis}, delta {delta}, v {actual_v}")
        if traveled_distance >= dis:
            break

def gy(dis, v):
    actual_v = v
    start_pos = node.odom.copy()
    global start_rot
    start_rot = node.rot.copy()
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
                actual_v = max(0.2, actual_v)
                # actual_v = max(0.1, actual_v)
            else:
                actual_v = min(-0.2, actual_v)
            node.get_logger().info("v"+str(actual_v))
        if traveled_distance >= dis:
            break



class RequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/up':
            self._handle_up()
        elif self.path == '/down':
            self._handle_down()
        elif self.path == '/left':
            self._handle_left()
        elif self.path == '/right':
            self._handle_right()
        elif self.path == '/turn_left':
            self._handle_turn_left()
        elif self.path == '/turn_right':
            self._handle_turn_right()
        elif self.path == '/heart':
            self._handle_heart()
        else:
            self.send_error(405)
            self.send_header('Content-Type','text/plain')
            self.end_headers()
            self.wfile.write(b"Method Not Allowed")
    
    def _respond_ok(self):
        self.send_response(200)
        self.send_header('Content-Type','text/plain')
        self.end_headers()
        self.wfile.write(b'OK')
        self.wfile.flush()
    
    def _respond_error(self, msg):
        self.send_error(400)
        self.send_header('Content-Type','text/plain')
        self.end_headers()
        self.wfile.write(msg.encode('utf-8'))
    
    def _handle_up(self):
        global node, IS_RUNNING
        task_done = Event()
        node.get_logger().info('Received /up request')
        
        def task():
            global IS_RUNNING
            IS_RUNNING = True
            try:
                gx(0.2, 0.4)
                do_delay(node.spc.stop_move, 0.2)()
            finally:
                IS_RUNNING = False
                task_done.set()

        task_added = False
        with TASK_LOCK:
            if not IS_RUNNING:
                TASK_QUEUE.append(task)
                task_added = True
        
        if task_added:
            task_done.wait()
            self._respond_ok()
        else:
            self._respond_error('Busy')



    def _handle_down(self):
        global node, IS_RUNNING
        task_done = Event()
        node.get_logger().info('Received /down request')
        
        def task():
            global IS_RUNNING
            IS_RUNNING = True
            try:
                gx(0.2, -0.4)
                do_delay(node.spc.stop_move, 0.2)()
            finally:
                IS_RUNNING = False
                task_done.set()

        task_added = False
        with TASK_LOCK:
            if not IS_RUNNING:
                TASK_QUEUE.append(task)
                task_added = True
        
        if task_added:
            task_done.wait()
            self._respond_ok()
        else:
            self._respond_error('Busy')

    def _handle_left(self):
        global node, IS_RUNNING
        task_done = Event()
        node.get_logger().info('Received /left request')
        
        def task():
            global IS_RUNNING
            IS_RUNNING = True
            try:
                gy(0.2, 0.4)
                do_delay(node.spc.stop_move, 0.2)()
            finally:
                IS_RUNNING = False
                task_done.set()

        task_added = False
        with TASK_LOCK:
            if not IS_RUNNING:
                TASK_QUEUE.append(task)
                task_added = True
        
        if task_added:
            task_done.wait()
            self._respond_ok()
        else:
            self._respond_error('Busy')

    def _handle_right(self):
        global node, IS_RUNNING
        task_done = Event()
        node.get_logger().info('Received /right request')
        
        def task():
            global IS_RUNNING
            IS_RUNNING = True
            try:
                gy(0.2, -0.4)
                do_delay(node.spc.stop_move, 0.2)()
            finally:
                IS_RUNNING = False
                task_done.set()

        task_added = False
        with TASK_LOCK:
            if not IS_RUNNING:
                TASK_QUEUE.append(task)
                task_added = True
        
        if task_added:
            task_done.wait()
            self._respond_ok()
        else:
            self._respond_error('Busy')

    def _handle_turn_left(self):
        # 向左的速度为正
        global node, IS_RUNNING
        task_done = Event()
        node.get_logger().info('Received /turn_left request')
        v_yaw = math.pi / 8 *3
        
        def task():
            global IS_RUNNING
            IS_RUNNING = True
            try:
                change_yaw(math.pi / 2 *1.07, v_yaw)
                do_delay(node.spc.stop_move, 0.2)()
            finally:
                IS_RUNNING = False
                task_done.set()

        task_added = False
        with TASK_LOCK:
            if not IS_RUNNING:
                TASK_QUEUE.append(task)
                task_added = True
        
        if task_added:
            task_done.wait()
            self._respond_ok()
        else:
            self._respond_error('Busy')
    
    def _handle_turn_right(self):
        global node, IS_RUNNING
        task_done = Event()
        node.get_logger().info('Received /turn_right request')
        v_yaw = math.pi / 8 *3
        
        def task():
            global IS_RUNNING
            IS_RUNNING = True
            try:
                change_yaw(math.pi / 2 *1.07, -v_yaw)
                do_delay(node.spc.stop_move, 0.2)()
            finally:
                IS_RUNNING = False
                task_done.set()

        task_added = False
        with TASK_LOCK:
            if not IS_RUNNING:
                TASK_QUEUE.append(task)
                task_added = True
        
        if task_added:
            task_done.wait()
            self._respond_ok()
        else:
            self._respond_error('Busy')

    def _handle_heart(self):
        # 由于比心动作会撞到，所以修改为挥手动作
        global node, IS_RUNNING
        node.get_logger().info('Received /heart request')
        task_done = Event()

        def task():
            global IS_RUNNING
            IS_RUNNING = True
            try:
                do_delay(node.spc.hello, 0.2)()
                do_delay(node.spc.stop_move, 0.2)()
            finally:
                IS_RUNNING = False
                task_done.set()

        task_added = False
        with TASK_LOCK:
            if not IS_RUNNING:
                TASK_QUEUE.append(task)
                task_added = True
        
        if task_added:
            task_done.wait()
            self._respond_ok()
        else:
            self._respond_error('Busy')

def exec_tasks():
    while True:
        with TASK_LOCK:
            if TASK_QUEUE:
                task = TASK_QUEUE.pop(0)
                task()
        time.sleep(0.1)

def main():
    rclpy.init()

    global node
    node = InteractionExperience()

    srv = HTTPServer(('0.0.0.0',DOG_INTERACT_PORT), RequestHandler)
    node.get_logger().info('HTTP服务器启动：http://0.0.0.0:8080')

    # 运行节点
    try:
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
        spin_thread.start()
        task_thread = threading.Thread(target=exec_tasks)
        task_thread.start()
        print('Starting server, use <Ctrl-C> to stop')
        srv.serve_forever()
    except KeyboardInterrupt:
        node.get_logger().info("接收到中断信号，停止节点")
    except Exception as e:
        # 处理其他所有普通异常
        node.get_logger().error(f"程序运行出现异常，即将终止：{str(e)}")
    finally:
        srv.server_close()
        # 销毁节点
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()