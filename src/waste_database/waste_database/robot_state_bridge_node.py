#!/usr/bin/env python3
"""
robot_state_bridge_node.py

Bridges ONE robot's real Nav2 odometry into the shared waste database's
expected /robot_state topic (waste_interfaces/RobotState), AND bridges
incoming ArUco detection logs to the /waste_database/register_detection service.
"""

import json
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
from waste_interfaces.msg import RobotState
from geometry_msgs.msg import Point
from waste_interfaces.srv import RegisterDetection


class RobotStateBridgeNode(Node):

    def __init__(self):
        super().__init__('robot_state_bridge_node')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('robot_id', 1)
        self.declare_parameter('odom_topic', '/robot1/odom')
        self.declare_parameter('battery_drain_per_meter', 0.3)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('move_speed_threshold', 0.02)

        self.robot_name = self.get_parameter('robot_name').value
        self.robot_id = self.get_parameter('robot_id').value
        odom_topic = self.get_parameter('odom_topic').value
        self.drain_per_meter = self.get_parameter('battery_drain_per_meter').value
        publish_rate = self.get_parameter('publish_rate_hz').value
        self.move_threshold = self.get_parameter('move_speed_threshold').value

        self.latest_odom = None
        self.last_position = None
        self.battery_level = 100.0
        self.current_task_id = -1

        # ── ODOMETRY & TASK SUBSCRIPTIONS ───────────────────────
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.create_subscription(
            Int32, f'{self.robot_name}/current_task_id', self.task_id_callback, 10)
        self.state_pub = self.create_publisher(RobotState, '/robot_state', 10)
        self.create_timer(1.0 / publish_rate, self.publish_state)

        # ── ARUCO DETECTION BRIDGE SETUP ────────────────────────
        # Handles both absolute and relative topic names gracefully
        detection_topic = f'/{self.robot_name}/aruco_detections/log' if not self.robot_name.startswith('/') else f'{self.robot_name}/aruco_detections/log'
        
        self.create_subscription(
            String,
            detection_topic,
            self.detection_log_callback,
            10
        )

        # Service Client for register_detection
        self.register_cli = self.create_client(
            RegisterDetection,
            '/waste_database/register_detection'
        )

        self.get_logger().info(
            f'robot_state_bridge_node up for "{self.robot_name}" '
            f'(id={self.robot_id}), reading {odom_topic} & {detection_topic}, publishing /robot_state.'
        )

    def task_id_callback(self, msg: Int32):
        self.current_task_id = msg.data

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        if self.last_position is not None:
            dist = math.hypot(pos.x - self.last_position[0], pos.y - self.last_position[1])
            self.battery_level = max(5.0, self.battery_level - dist * self.drain_per_meter)
        self.last_position = (pos.x, pos.y)
        self.latest_odom = msg

    def publish_state(self):
        if self.latest_odom is None:
            return  # stay silent until first odom arrives

        msg = RobotState()
        msg.robot_name = self.robot_name
        msg.robot_id = self.robot_id
        msg.pose = self.latest_odom.pose.pose
        msg.current_task_id = self.current_task_id
        msg.battery_level = float(self.battery_level)
        msg.last_update = self.get_clock().now().to_msg()

        twist = self.latest_odom.twist.twist
        speed = math.hypot(twist.linear.x, twist.linear.y) + abs(twist.angular.z)
        msg.status = 'NAVIGATING' if speed > self.move_threshold else 'IDLE'

        self.state_pub.publish(msg)

    def detection_log_callback(self, msg: String):
        """
        Parses incoming detection JSON and calls register_detection service.
        Safely handles missing fields without throwing exceptions.
        """
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse detection JSON: {e}")
            return

        if not self.register_cli.service_is_ready():
            self.get_logger().warn(
                "RegisterDetection service not ready, dropping detection...",
                throttle_duration_sec=3.0
            )
            return

        req = RegisterDetection.Request()
        req.marker_id = int(data.get('marker_id', 0))

        # Build nested geometry_msgs/Point object
        req.position = Point()
        req.position.x = float(data.get('map_x', 0.0))
        req.position.y = float(data.get('map_y', 0.0))
        req.position.z = float(data.get('map_z', 0.0))

        # Additional metadata fields
        req.waste_type = str(data.get('waste_type', 'unknown'))
        req.confidence = float(data.get('confidence', 1.0))
        
        # Flexibly accepts 'robot' or 'robot_name' key from JSON
        req.robot_name = str(data.get('robot', data.get('robot_name', self.robot_name)))

        self.get_logger().info(
            f"Forwarding ArUco ID:{req.marker_id} from {req.robot_name} to database service..."
        )
        self.register_cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
