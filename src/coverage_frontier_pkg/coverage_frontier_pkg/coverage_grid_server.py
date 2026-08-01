#!/usr/bin/env python3
"""
coverage_grid_server.py

Owns the ONE shared coverage grid for the whole building. This is the
key architectural choice that makes collaboration/reallocation possible:
if each robot kept its own local coverage grid, they'd have no way to
know what the others have already covered. A single shared server
avoids that sync problem entirely -- every robot queries the same
source of truth.

Responsibilities:
  - Load the occupancy map (pgm/yaml) and room polygons (rooms.yaml).
  - Subscribe to each robot's live pose and mark cells within
    detection_range_m as "seen" automatically, continuously -- no robot
    needs to explicitly report what it has covered.
  - Serve QueryNearestUnseen: given a robot's position and a room
    filter (or global), return the nearest still-unseen free cell.
    This IS the frontier query -- recomputed fresh on every call, never
    cached as a plan.
  - Serve GetRoomStatus: unseen-cell counts per room, used by robot
    nodes to decide whether to reallocate to another room.

This node holds no knowledge of Nav2, navigation, or spinning -- it is
purely the shared coverage-state bookkeeper.
"""

import math
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt, label as cc_label

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry

from coverage_frontier_interfaces.srv import QueryNearestUnseen, GetRoomStatus


class CoverageGridServer(Node):
    def __init__(self):
        super().__init__("coverage_grid_server")

        self.declare_parameter("map_pgm_path", "")
        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("rooms_yaml_path", "")
        self.declare_parameter("pose_topic_suffix", "/amcl_pose")  # per-robot: /<robot_id>/amcl_pose
        self.declare_parameter("robot_ids", ["robot_1", "robot_2", "robot_3"])

        map_pgm = self.get_parameter("map_pgm_path").value
        map_yaml = self.get_parameter("map_yaml_path").value
        rooms_yaml = self.get_parameter("rooms_yaml_path").value
        pose_suffix = self.get_parameter("pose_topic_suffix").value
        robot_ids = list(self.get_parameter("robot_ids").value)

        with open(rooms_yaml) as f:
            cfg = yaml.safe_load(f)
        self.clearance_m = float(cfg.get("clearance_m", 0.5))
        self.detection_range_m = float(cfg.get("detection_range_m", 2.5))
        self.room_defs = cfg["rooms"]

        self._load_map(map_pgm, map_yaml)
        self._build_room_masks()

        # coverage[room_name] = bool array (same shape as map), True = seen
        self.coverage = {r: np.zeros_like(self.traversable, dtype=bool) for r in self.room_defs}
        # cells that are walls/obstacles count as "seen" trivially so they
        # never show up as an unseen target
        for r in self.coverage:
            self.coverage[r] |= ~self.free_eroded

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self._pose_subs = []
        for rid in robot_ids:
            topic = f"/{rid}{pose_suffix}"
            sub = self.create_subscription(
                PoseWithCovarianceStamped, topic,
                lambda msg, rid=rid: self._on_pose(rid, msg.pose.pose.position.x, msg.pose.pose.position.y),
                qos,
            )
            self._pose_subs.append(sub)
            self.get_logger().info(f"Subscribed to {topic} for live coverage marking")

        self.create_service(QueryNearestUnseen, "query_nearest_unseen", self._on_query_nearest)
        self.create_service(GetRoomStatus, "get_room_status", self._on_get_status)

        self.get_logger().info(
            f"CoverageGridServer up. rooms={list(self.room_defs.keys())} "
            f"clearance={self.clearance_m}m detection_range={self.detection_range_m}m"
        )

    # ---------------- map / room setup ----------------

    def _load_map(self, pgm_path, yaml_path):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        img = np.array(Image.open(pgm_path))
        if meta.get("negate", 0):
            img = 255 - img
        res = meta["resolution"]
        occ_thresh = meta.get("occupied_thresh", 0.65)
        free_thresh = meta.get("free_thresh", 0.25)
        occ_prob = (255.0 - img) / 255.0
        traversable = (occ_prob < free_thresh) & ~(occ_prob > occ_thresh)
        traversable[0, :] = False
        traversable[-1, :] = False
        traversable[:, 0] = False
        traversable[:, -1] = False

        self.resolution = res
        self.origin = meta["origin"]
        self.shape = img.shape
        self.traversable = traversable
        dist = distance_transform_edt(traversable)
        self.free_eroded = dist >= (self.clearance_m / res)

    def world_to_px(self, x, y):
        h = self.shape[0]
        col = int(round((x - self.origin[0]) / self.resolution))
        row = int(round(h - 1 - (y - self.origin[1]) / self.resolution))
        return row, col

    def px_to_world(self, r, c):
        h = self.shape[0]
        x = c * self.resolution + self.origin[0]
        y = (h - 1 - r) * self.resolution + self.origin[1]
        return x, y

    def _rasterize_room(self, rects):
        h, w = self.shape
        mask = np.zeros((h, w), dtype=bool)
        for xmin, xmax, ymin, ymax in rects:
            r0, c0 = self.world_to_px(xmin, ymax)
            r1, c1 = self.world_to_px(xmax, ymin)
            r0, r1 = sorted((max(0, r0), min(h, r1)))
            c0, c1 = sorted((max(0, c0), min(w, c1)))
            mask[r0:r1, c0:c1] = True
        return mask

    def _build_room_masks(self):
        self.room_masks = {}
        for name, spec in self.room_defs.items():
            rects = spec["rects"]
            room_mask = self._rasterize_room(rects)
            free_in_room = room_mask & self.free_eroded
            spawn_x, spawn_y = spec["home_spawn"]
            seed_rc = self.world_to_px(spawn_x, spawn_y)
            lbl, n = cc_label(free_in_room, structure=np.ones((3, 3)))
            r, c = seed_rc
            comp = lbl[r, c] if free_in_room[r, c] else 0
            if comp == 0:
                ys, xs = np.where(lbl > 0)
                d = (ys - r) ** 2 + (xs - c) ** 2
                comp = lbl[ys[np.argmin(d)], xs[np.argmin(d)]] if len(ys) else 0
            reachable = (lbl == comp) if comp != 0 else free_in_room
            self.room_masks[name] = reachable
            self.get_logger().info(f"Room '{name}': {reachable.sum()} reachable free cells")

    # ---------------- live coverage marking ----------------

    def _on_pose(self, robot_id, x, y):
        r_px = int(round(self.detection_range_m / self.resolution))
        cr, cc = self.world_to_px(x, y)
        r0, r1 = max(0, cr - r_px), min(self.shape[0], cr + r_px + 1)
        c0, c1 = max(0, cc - r_px), min(self.shape[1], cc + r_px + 1)
        yy, xx = np.ogrid[r0:r1, c0:c1]
        disk = (yy - cr) ** 2 + (xx - cc) ** 2 <= r_px ** 2
        for room, mask in self.room_masks.items():
            sub = mask[r0:r1, c0:c1]
            self.coverage[room][r0:r1, c0:c1] |= (disk & sub)

    # ---------------- services ----------------

    def _nearest_unseen_in_room(self, room, x, y):
        seen = self.coverage[room]
        mask = self.room_masks[room]
        unseen = mask & ~seen
        count = int(unseen.sum())
        if count == 0:
            return None, 0
        ys, xs = np.where(unseen)
        r0, c0 = self.world_to_px(x, y)
        d2 = (ys - r0) ** 2 + (xs - c0) ** 2
        i = int(np.argmin(d2))
        wx, wy = self.px_to_world(int(ys[i]), int(xs[i]))
        return (wx, wy), count

    def _on_query_nearest(self, request, response):
        room_filter = request.room_name
        rooms_to_try = [room_filter] if room_filter else list(self.room_defs.keys())

        best = None
        best_room = None
        best_d = float("inf")
        counts = {}
        for room in rooms_to_try:
            target, count = self._nearest_unseen_in_room(room, request.robot_x, request.robot_y)
            counts[room] = count
            if target is not None:
                d = math.hypot(target[0] - request.robot_x, target[1] - request.robot_y)
                if d < best_d:
                    best_d = d
                    best = target
                    best_room = room

        global_unseen = sum(int((self.room_masks[r] & ~self.coverage[r]).sum()) for r in self.room_defs)

        if best is None:
            response.found = False
            response.target_x = 0.0
            response.target_y = 0.0
            response.found_in_room = ""
            response.unseen_count_in_room = 0
            response.unseen_count_global = global_unseen
            return response

        response.found = True
        response.target_x, response.target_y = best
        response.found_in_room = best_room
        response.unseen_count_in_room = counts.get(best_room, 0)
        response.unseen_count_global = global_unseen
        return response

    def _on_get_status(self, request, response):
        rooms = [request.room_name] if request.room_name else list(self.room_defs.keys())
        names, counts = [], []
        for r in rooms:
            c = int((self.room_masks[r] & ~self.coverage[r]).sum())
            names.append(r)
            counts.append(c)
        response.room_names = names
        response.unseen_counts = counts
        response.global_unseen_count = sum(
            int((self.room_masks[r] & ~self.coverage[r]).sum()) for r in self.room_defs
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CoverageGridServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
