import rclpy
from rclpy.node import Node
import time
import math
import os
import threading
import json

from clients.obstacles_avoid_client import ObstaclesAvoidClient
from clients.sport_client import SportClient
from clients.utils import *
from http.server import BaseHTTPRequestHandler, HTTPServer
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
import numpy as np
from clients.vui_client import VUIClient

node = None

class NavStubbing(Node):
    def __init__(self):
        super().__init__('get_state_node')
        self.state = "unknown"

        self.spc = SportClient(self)
        self.oac = ObstaclesAvoidClient(self)
        self.vuc = VUIClient(self)

        self.pose_sub = self.create_subscription(
                    msg_type=PoseStamped,
                    topic='/utlidar/robot_pose',
                    callback=self.handle_pose,
                    qos_profile=10
                )
        self.rot = [0, 0, 0]

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

TASK_QUEUE = []
TASK_LOCK = threading.Lock()

class RequestHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/map':
            self._handle_map()
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
    def _handle_map(self):
        content_length = int(self.headers['Content-Length'])
        post_data = self.rfile.read(content_length)
        try:
            json_data = json.loads(post_data.decode('utf-8'))
            map = json_data.get('map')
            map_id = json_data.get('map_id')
            
            print(f'map id: {map_id}')
            print(f'map: {map}')

            global node
            start_rot = node.rot

            # def move(x, y):
            #     global node
            #     spc = node.spc
            #     oac = node.oac

            #     rectify = math.pi/36
            #     delta = node.rot[2] - start_rot[2]

            #     if delta > rectify:
            #         spc.move(x, y, -rectify)
            #     elif delta < -rectify:
            #         spc.move(x, y, rectify)
            #     else:
            #         spc.move(x, y, 0)

            def task():
                def move(x, y):
                    global node
                    spc = node.spc
                    oac = node.oac

                    # rectify = math.pi/36
                    rectify = math.pi/12
                    delta = node.rot[2] - start_rot[2]
                    
                    # 处理角度跨越±π的情况
                    while delta > math.pi:
                        delta -= 2 * math.pi
                    while delta < -math.pi:
                        delta += 2 * math.pi

                    gain = 1.3  # 增益系数，可调整
                    
                    # 计算角度修正量，使用比例控制
                    angular_correction = 0
                    if abs(delta) > rectify / 36:
                        angular_correction = -delta * gain 
                        # 限制最大修正量
                        angular_correction = max(-rectify, min(rectify, angular_correction))
                    
                    spc.move(x, y, angular_correction)
                    print(delta*180)

                if map is not None and map_id is not None:
                    spc = node.spc
                    oac = node.oac
                    vuc = node.vuc
                    print('Start navigation on map ', map_id)
                    if int(map_id) == 1:
                        # do_delay(spc.stand_up, 1)()
                        do_delay(spc.balanced_stand, 1)()
                        do_delay(oac.disable, 0.1)()
                        # do_delay(oac.use_api, 1)(True)

                        # do_delay(oac.move_to_relative, 3)(0, 0, -math.pi/2)
                        # do_delay(oac.move_to_relative, 3)(1.25, 0, 0)
                        # do_delay(oac.move_to_relative, 3)(0, 0, math.pi/2)
                        # do_delay(oac.move_to_relative, 6)(2, 0, 0)
                        # do_delay(oac.move_to_relative, 3)(0, 0, -math.pi/2)
                        # do_delay(oac.move_to_relative, 3)(1, 0, 0)
                        # do_delay(oac.move_to_relative, 3)(0, 0, math.pi/2)
                        # do_delay(oac.move_to_relative, 3)(1, 0, 0)
                        # do_delay(oac.move_to_relative, 3)(0, 0, -math.pi/2)
                        # do_delay(oac.move_to_relative, 3)(0.5, 0, 0)
                        # do_delay(oac.move_to_absolute, 10)(0, 1, 0)

                        # do_delay(oac.move_to_relative, 2.5)(0, -1.25, 0)
                        # do_delay(oac.move_to_relative, 6)(2, 0, 0)
                        # do_delay(oac.move_to_relative, 2)(0, -1, 0)
                        # do_delay(oac.move_to_relative, 3)(2, 0, 0)

                        # repeat(spc.move, 20, 0.1)(0.0, -0.5, -math.pi/36)
                        # repeat(spc.move, 40, 0.1)(0.5, 0, 0)
                        # repeat(spc.move, 25, 0.1)(0, -0.5, -math.pi/36)
                        # repeat(spc.move, 20, 0.1)(0.5, 0, 0)
                        # repeat(spc.move, 38, 0.1)(0.5, 0, 0)
                        # repeat(spc.move, 15, 0.1)(0, -0.5, -math.pi/36)

                        repeat(move, 20, 0.1)(0.0, -0.5)
                        # do_delay(vuc.set_brightness, 0.1)(6.0)
                        do_delay(spc.balanced_stand, 5)()
                        # do_delay(vuc.set_brightness, 0.1)(0.0)
                        repeat(move, 40, 0.1)(0.5, 0)
                        do_delay(spc.balanced_stand, 1)()
                        repeat(move, 20, 0.1)(0, -0.5)
                        repeat(move, 20, 0.1)(0.5, 0)
                        repeat(move, 38, 0.1)(0.5, 0)
                        do_delay(spc.balanced_stand, 1)()
                        repeat(move, 15, 0.1)(0, -0.5)

                        do_delay(spc.stop_move, 1)()
                        do_delay(spc.balanced_stand, 1)()

                        # do_delay(oac.use_api, 1)(False)
                        node.get_logger().info('Navigation completed')

                    elif int(map_id) == 2:
                        do_delay(spc.balanced_stand, 1)()
                        do_delay(oac.disable, 0.1)()

                        repeat(move, 40, 0.1)(0.5, 0)
                        # do_delay(vuc.set_brightness, 0.1)(6.0)
                        do_delay(spc.balanced_stand, 5)()
                        repeat(move, 20, 0.1)(0.0, -0.5)

                        do_delay(spc.balanced_stand, 1)()
                        repeat(move, 20, 0.1)(0, -0.5)
                        repeat(move, 20, 0.1)(0.5, 0)
                        repeat(move, 38, 0.1)(0.5, 0)
                        do_delay(spc.balanced_stand, 1)()
                        repeat(move, 15, 0.1)(0, -0.5)

                        do_delay(spc.stop_move, 1)()
                        do_delay(spc.balanced_stand, 1)()

                        node.get_logger().info('Navigation completed')


            with TASK_LOCK:
                if len(TASK_QUEUE) == 0:
                    TASK_QUEUE.append(task)
                else:
                    self._response_error('Busy')
            
            self._respond_ok()

        except json.JSONDecodeError as e:
            self._response_error(f'Invalid JSON: {e}')

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
    node = NavStubbing()
    srv = HTTPServer(('0.0.0.0',10066), RequestHandler)
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