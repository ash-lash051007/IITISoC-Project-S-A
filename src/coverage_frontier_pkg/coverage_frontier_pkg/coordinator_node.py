#!/usr/bin/env python3
"""
coordinator_node.py

Lightweight -- does not command any robot directly. Two jobs only:
  1. Aggregate unique ArUco IDs detected across all 3 robots (each
     robot's existing, untouched ArUco detector publishes to its own
     topic; this node just subscribes to all three).
  2. Poll the shared coverage grid's global unseen-cell count.

Publishes /mission_complete (std_msgs/Bool) the instant EITHER all
expected marker IDs have been seen OR the global unseen count hits
zero -- whichever happens first. Robots are expected to subscribe to
this topic and cancel their current nav goal + stop querying once it
fires (that subscription is a ~5 line addition to robot_coverage_node,
intentionally left as a hook rather than baked in, so you can decide
whether "stop immediately" or "finish current goal then stop" fits
your grading/demo needs better).
"""

import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from coverage_frontier_interfaces.srv import GetRoomStatus


class CoordinatorNode(Node):
    def __init__(self):
        super().__init__("coordinator_node")

        self.declare_parameter("robot_ids", ["robot_1", "robot_2", "robot_3"])
        self.declare_parameter("detection_topic_suffix", "/aruco_detections")  # publishes std_msgs/String marker id
        self.declare_parameter("expected_marker_ids", [
            "aruco_0", "aruco_1", "aruco_2", "aruco_3", "aruco_4", "aruco_5",
            "aruco_6", "aruco_7", "aruco_8", "aruco_9", "aruco_10", "aruco_11",
        ])
        self.declare_parameter("poll_period_s", 2.0)

        robot_ids = list(self.get_parameter("robot_ids").value)
        suffix = self.get_parameter("detection_topic_suffix").value
        self.expected = set(self.get_parameter("expected_marker_ids").value)
        self.found = set()

        for rid in robot_ids:
            topic = f"/{rid}{suffix}"
            self.create_subscription(String, topic, self._on_detection, 10)
            self.get_logger().info(f"Coordinator subscribed to {topic}")

        self.status_cli = self.create_client(GetRoomStatus, "get_room_status")
        while not self.status_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Coordinator waiting for get_room_status service...")

        self.complete_pub = self.create_publisher(Bool, "/mission_complete", 10)
        self.declared_complete = False

        period = float(self.get_parameter("poll_period_s").value)
        self.create_timer(period, self._poll_coverage)

    def _on_detection(self, msg: String):
        marker_id = msg.data
        if marker_id not in self.found:
            self.found.add(marker_id)
            self.get_logger().info(f"Marker detected: {marker_id}  ({len(self.found)}/{len(self.expected)})")
            if self.expected and self.found >= self.expected:
                self._declare_complete("all markers found")

    def _poll_coverage(self):
        if self.declared_complete:
            return
        req = GetRoomStatus.Request()
        req.room_name = ""
        future = self.status_cli.call_async(req)
        future.add_done_callback(self._on_status)

    def _on_status(self, future):
        resp = future.result()
        if resp.global_unseen_count == 0:
            self._declare_complete("full coverage reached")
        else:
            self.get_logger().info(
                f"Coverage remaining: {resp.global_unseen_count} cells | "
                f"markers found: {len(self.found)}/{len(self.expected)}"
            )

    def _declare_complete(self, reason):
        if self.declared_complete:
            return
        self.declared_complete = True
        self.get_logger().info(f"MISSION COMPLETE ({reason}). Publishing /mission_complete.")
        msg = Bool()
        msg.data = True
        self.complete_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
