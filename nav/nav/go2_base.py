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
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry

class Go2BaseNode(Node):
    def __init__(self):
        super().__init__('go2_base_node')
        self.spc = SportClient(self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose_sub = self.create_subscription(
            msg_type=PoseStamped,
            topic='/utlidar/robot_pose',
            callback=self.handle_pose,
            qos_profile=10
        )

        # self.

        self.odom_pub = self.create_publisher(
            msg_type=PoseStamped,
            topic='/odom',
            qos_profile=10
        )

        self.cmd_vel_sub = self.create_subscription(
            msg_type=Twist,
            topic='/cmd_vel',
            callback=self.handle_speed,
            qos_profile=10,
        )

    def handle_speed(self, msg: Twist):
        self.spc.move(msg.linear.x, msg.linear.y, msg.angular.z)
        if (msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0):
            self.spc.stop_move()
        

    def handle_pose(self, msg: PoseStamped):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg() 
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)

        odom_msg = Odometry()
        odom_msg.header.stamp = t.header.stamp
        odom_msg.header.frame_id = t.header.frame_id
        odom_msg.child_frame_id = t.child_frame_id

        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Go2BaseNode()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()