#!/usr/bin/env python3
"""
robot_state_bridge_node.py

Bridges ONE robot's real Nav2 odometry into the shared waste database's
expected /robot_state topic (waste_interfaces/RobotState).

One instance of this node runs per robot (see robot_state_bridges.launch.py),
each given its own robot_name / robot_id / odom_topic via parameters.

Design notes:
  - Subscribes on the robot's own namespaced odom topic (e.g. /robot1/odom),
    but publishes on the ABSOLUTE shared topic /robot_state -- the database
    node is not namespaced, so this must be reachable from anywhere.
  - status is derived from real odometry: if the robot's linear/angular
    speed is above a small threshold it reports NAVIGATING, otherwise IDLE.
    This is a genuine signal, not a placeholder.
  - battery_level is simulated: it starts at 100 and drains proportionally
    to distance actually traveled (from consecutive odom readings), floored
    at 5 so it never hits zero and looks broken.
  - current_task_id stays at -1 here -- it will be set for real once the
    per-robot task-manager node (the next piece) is wired in.
  - Publishes nothing until the first odom message arrives, rather than
    broadcasting a fake pose at (0,0,0) before the robot is actually known.
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from waste_interfaces.msg import RobotState


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
        self.current_task_id = -1  # set for real once the task manager exists

        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.state_pub = self.create_publisher(RobotState, '/robot_state', 10)
        self.create_timer(1.0 / publish_rate, self.publish_state)

        self.get_logger().info(
            f'robot_state_bridge_node up for "{self.robot_name}" '
            f'(id={self.robot_id}), reading {odom_topic}, publishing /robot_state.'
        )

    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        if self.last_position is not None:
            dist = math.hypot(pos.x - self.last_position[0], pos.y - self.last_position[1])
            self.battery_level = max(5.0, self.battery_level - dist * self.drain_per_meter)
        self.last_position = (pos.x, pos.y)
        self.latest_odom = msg

    def publish_state(self):
        if self.latest_odom is None:
            return  # nothing received yet -- stay silent instead of faking a pose

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
