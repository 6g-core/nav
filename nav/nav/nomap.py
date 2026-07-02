from clients.sport_client import SportClient
from rclpy.node import Node
from nav_msgs.msg import Odometry
import rclpy

LIDAR_TOPIC_NAME = "/utlidar/cloud"
ODOM_TOPIC_NAME = "/utlidar/robot_odom"

class NoMapNode(Node):
    def __init__(self):
        super().__init__("nomap_node")
        self.sport_client = SportClient(self, "sport_client")

        self.create_subscription(Odometry, ODOM_TOPIC_NAME, self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        # Process odometry data
        self.get_logger().info(f"Received odometry data: {msg.pose.pose}")

def main(args=None):
    rclpy.init(args=args)
    node = NoMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

