#!/usr/bin/env python3
"""
aruco_database_bridge_node.py

Bridges ONE robot's real ArUco detections into the shared waste database.

Subscribes to <robot_name>/aruco_detections/log (std_msgs/String, JSON
payload published by aruco_detector.py) and calls
/waste_database/register_detection for each valid detection.

One instance runs per robot (see aruco_database_bridges.launch.py), each
given its own robot_name / log_topic via parameters.

Expected JSON payload (from aruco_detector.py):
  {
    "marker_id": int,
    "waste_type": str,      # "hazardous" / "recyclable" / "general"
    "map_x": float, "map_y": float, "map_z": float,
    "distance": float,
    "robot": str,
    "timestamp": float
  }

Design notes:
  - Marker ID is re-validated here (0-11) as a second safety net, in case
    an un-patched copy of aruco_detector.py is ever run -- this bridge
    should never forward an out-of-range ID to the database regardless of
    what the detector sends.
  - Confidence isn't provided by the detector (OpenCV's classic ArUco API
    doesn't emit one), so it's derived here from detection distance:
    closer detections have more pixels on the marker and less corner-
    localization noise, so confidence falls off linearly with distance and
    is clamped to [min_confidence, max_confidence].
  - Per-marker resend throttling: a marker sitting in camera view gets
    re-detected on every incoming camera frame (potentially 10-30Hz). Without
    throttling, that floods register_detection with redundant "duplicate,
    refine position" calls for the same marker many times per second. Each
    robot instance only forwards a given marker_id once per
    min_resend_interval_sec, still allowing periodic position refinement
    without spamming the service or the logs.
"""

import json
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Point
from waste_interfaces.srv import RegisterDetection

VALID_MARKER_IDS = set(range(12))  # 0-11 -- keep in sync with aruco_detector.py
REQUIRED_FIELDS = ('marker_id', 'waste_type', 'map_x', 'map_y', 'map_z', 'distance', 'robot')


class ArucoDatabaseBridgeNode(Node):

    def __init__(self):
        super().__init__('aruco_database_bridge_node')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('log_topic', '/robot1/aruco_detections/log')
        self.declare_parameter('max_distance', 10.0)
        self.declare_parameter('min_confidence', 0.3)
        self.declare_parameter('max_confidence', 0.99)
        self.declare_parameter('min_resend_interval_sec', 3.0)

        self.robot_name = self.get_parameter('robot_name').value
        log_topic = self.get_parameter('log_topic').value
        self.max_distance = self.get_parameter('max_distance').value
        self.min_confidence = self.get_parameter('min_confidence').value
        self.max_confidence = self.get_parameter('max_confidence').value
        self.min_resend_interval = self.get_parameter('min_resend_interval_sec').value

        self.last_sent = {}  # marker_id -> monotonic time it was last forwarded

        self.register_client = self.create_client(
            RegisterDetection, '/waste_database/register_detection')

        self.create_subscription(String, log_topic, self.detection_callback, 10)

        self.get_logger().info(
            f'aruco_database_bridge_node up for "{self.robot_name}", '
            f'reading {log_topic}, calling /waste_database/register_detection.'
        )

    def distance_to_confidence(self, distance):
        frac = 1.0 - (distance / self.max_distance)
        frac = max(0.0, min(1.0, frac))
        return self.min_confidence + frac * (self.max_confidence - self.min_confidence)

    def detection_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Malformed detection JSON from {self.robot_name}: {e}')
            return

        missing = [k for k in REQUIRED_FIELDS if k not in data]
        if missing:
            self.get_logger().error(
                f'Detection JSON from {self.robot_name} missing fields {missing}: {data}')
            return

        try:
            marker_id = int(data['marker_id'])
        except (TypeError, ValueError):
            self.get_logger().error(f'Non-integer marker_id from {self.robot_name}: {data}')
            return

        if marker_id not in VALID_MARKER_IDS:
            self.get_logger().warn(
                f'Dropping detection with marker_id {marker_id} outside valid '
                f'range 0-11 (from {self.robot_name}).', throttle_duration_sec=2.0)
            return

        now = time.monotonic()
        last = self.last_sent.get(marker_id)
        if last is not None and (now - last) < self.min_resend_interval:
            return  # seen too recently, skip -- avoids per-frame spam
        self.last_sent[marker_id] = now

        if not self.register_client.service_is_ready():
            self.get_logger().warn(
                '/waste_database/register_detection not available yet, dropping detection.',
                throttle_duration_sec=5.0)
            return

        confidence = self.distance_to_confidence(float(data['distance']))

        req = RegisterDetection.Request()
        req.marker_id = marker_id
        req.position = Point(
            x=float(data['map_x']), y=float(data['map_y']), z=float(data['map_z']))
        req.waste_type = str(data['waste_type'])
        req.confidence = float(confidence)
        req.robot_name = str(data['robot'])

        self.register_client.call_async(req).add_done_callback(self.on_register_response)

    def on_register_response(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'register_detection call failed: {e}')
            return
        if not resp.accepted:
            self.get_logger().error(f'register_detection rejected: {resp.message}')
        # Accepted responses are already logged (colored) by the database node itself.


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDatabaseBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
