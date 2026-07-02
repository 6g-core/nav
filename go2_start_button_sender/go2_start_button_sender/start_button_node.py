import rclpy
from rclpy.node import Node
from unitree_go.msg import WirelessController
import json
import http.client

class StartButtonSender(Node):
    def __init__(self):
        super().__init__('start_button_sender')
        
        # ==================== 用户配置区域 ====================
        # 双击目标
        self.target_ip_double = "192.168.123.100"
        self.target_port_double = 18891
        
        # 长按目标
        self.target_ip_long = "192.168.123.100"
        self.target_port_long = 18891
        
        self.start_mask = 0x04               # START 键位掩码 (实测=4)
        self.double_click_window = 0.5         # 双击时间窗口：500ms
        self.long_press_duration = 2.0         # 长按触发时间：2秒
        # ======================================================
        
        # 状态变量
        self.prev_pressed = False
        self.press_start_time = None         # 当前按住的起始时间
        self.long_triggered = False          # 本次按住是否已触发过长按
        
        self.first_click_time = None         # 第一次点击松开的时间
        self.waiting_second_click = False    # 是否在等待第二次点击
        
        self.sub = self.create_subscription(
            WirelessController,
            '/wirelesscontroller',
            self.controller_callback,
            10
        )
        
        self.get_logger().info(
            f"已启动！监听 /wirelesscontroller，START 键掩码: {self.start_mask}"
        )
        self.get_logger().info(
            f"双击: {self.double_click_window*1000:.0f}ms 内按两次 -> "
            f"http://{self.target_ip_double}:{self.target_port_double}/api/v1/exec"
        )
        self.get_logger().info(
            f"长按: 按住 {self.long_press_duration}s -> "
            f"http://{self.target_ip_long}:{self.target_port_long}/api/v1/exec"
        )

    def controller_callback(self, msg: WirelessController):
        is_pressed = (msg.keys & self.start_mask) != 0
        was_pressed = self.prev_pressed
        
        current_time = self.get_clock().now()
        current_sec = current_time.nanoseconds / 1e9
        
        # === 上升沿：刚按下 ===
        if is_pressed and not was_pressed:
            if self.waiting_second_click and self.first_click_time is not None:
                elapsed = current_sec - self.first_click_time
                if elapsed <= self.double_click_window:
                    self.get_logger().info(
                        f">>> 双击 START 触发！间隔: {elapsed*1000:.0f}ms <<<"
                    )
                    self.send_http_double_click()
                    # 重置所有状态
                    self.waiting_second_click = False
                    self.first_click_time = None
                    self.press_start_time = None
                    self.long_triggered = False
                else:
                    self.get_logger().info(
                        f"双击窗口超时 ({elapsed*1000:.0f}ms)，视为新按下"
                    )
                    self.press_start_time = current_sec
                    self.long_triggered = False
                    self.waiting_second_click = False
                    self.first_click_time = None
            else:
                # 新的按住
                self.press_start_time = current_sec
                self.long_triggered = False
        
        # === 持续按住 ===
        elif is_pressed and was_pressed:
            if self.press_start_time is not None and not self.long_triggered:
                duration = current_sec - self.press_start_time
                if duration >= self.long_press_duration:
                    self.get_logger().info(
                        f">>> 长按 START {duration:.1f} 秒，触发 <<<"
                    )
                    self.send_http_long_press()
                    self.long_triggered = True
                    # 长按触发后，取消双击等待，避免松开后误判
                    self.waiting_second_click = False
                    self.first_click_time = None
        
        # === 下降沿：刚松开 ===
        elif not is_pressed and was_pressed:
            if self.long_triggered:
                self.get_logger().info("长按后松开，重置状态")
                self.press_start_time = None
                self.long_triggered = False
                self.waiting_second_click = False
                self.first_click_time = None
            else:
                # 未触发长按，记录第一次点击
                if self.press_start_time is not None:
                    self.first_click_time = current_sec
                    self.waiting_second_click = True
                    self.get_logger().info("第一次点击松开，等待第二次...")
                self.press_start_time = None
        
        # === 未按住：检查双击超时 ===
        else:
            if self.waiting_second_click and self.first_click_time is not None:
                elapsed = current_sec - self.first_click_time
                if elapsed > self.double_click_window:
                    self.get_logger().info(
                        f"双击等待超时 ({elapsed*1000:.0f}ms)，重置"
                    )
                    self.waiting_second_click = False
                    self.first_click_time = None
        
        self.prev_pressed = is_pressed

    def send_http_double_click(self):
        """双击：发送 curl 等效消息到 192.168.123.100:18891 /api/v1/exec"""
        body = '{"target":"white toy rabbit"}'
        self._send_http(
            host=self.target_ip_double,
            port=self.target_port_double,
            path="/api/v1/exec",
            body=body,
            headers={"Content-Type": "application/json"},
            timeout=120.0  # --max-time 120
        )

    def send_http_long_press(self):
        """长按 2 秒：发送 curl 等效消息到 192.168.123.99:18891 /api/v1/exec"""
        body = '{"input":"/home/ubuntu/marm_demo_auto/src/control/d1_pose0.json","debug":false,"log":true}'
        self._send_http(
            host=self.target_ip_long,
            port=self.target_port_long,
            path="/api/v1/exec",
            body=body,
            headers={"Content-Type": "application"}
        )

    def _send_http(self, host, port, path, body, headers, timeout=2.0):
        try:
            conn = http.client.HTTPConnection(host, port, timeout=timeout)
            h = dict(headers)
            h["Content-Length"] = str(len(body))
            h["Connection"] = "close"
            conn.request("POST", path, body=body, headers=h)
            response = conn.getresponse()
            self.get_logger().info(
                f"HTTP 成功 [{host}:{port}]: {response.status} {response.reason}"
            )
            conn.close()
        except Exception as e:
            self.get_logger().error(f"HTTP 失败 [{host}:{port}]: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StartButtonSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
