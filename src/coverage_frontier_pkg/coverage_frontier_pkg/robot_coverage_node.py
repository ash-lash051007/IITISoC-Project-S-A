#!/usr/bin/env python3
import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from nav2_msgs.action import NavigateToPose, Spin
from std_msgs.msg import Bool

# Coverage Interfaces
from coverage_frontier_interfaces.srv import QueryNearestUnseen, GetRoomStatus, ReserveDoor

# Task Allocator Messages
from task_allocator_msgs.msg import TaskAssignment, RobotState


class RobotCoverageNode(Node):
    def __init__(self):
        super().__init__("robot_coverage_node")

        self.declare_parameter("robot_id", "robot1")
        self.declare_parameter("home_room", "warehouse")
        self.declare_parameter("rooms_yaml_path", "")
        self.declare_parameter("spin_interval_m", 2.5)

        self.robot_id = self.get_parameter("robot_id").value
        # Parse numerical ID from string (e.g. "robot_1" or "robot1" -> 1)
        self.robot_num_id = int(''.join(filter(str.isdigit, self.robot_id)) or 1)
        
        self.current_room = self.get_parameter("home_room").value
        self.spin_interval_m = float(self.get_parameter("spin_interval_m").value)

        rooms_yaml = self.get_parameter("rooms_yaml_path").value
        with open(rooms_yaml) as f:
            cfg = yaml.safe_load(f)
        self.doors = cfg["doors"]

        self.pose_x, self.pose_y = None, None
        self.dist_since_spin = 0.0
        self._last_pose_for_dist = None
        self.finished = False
        self._busy = False
        self._warned_no_pose = False

        # --- MODE SWITCHING & TASK ALLOCATOR STATES ---
        self.mode = "SEARCH"            # "SEARCH" or "COLLECT"
        self.current_target_id = -1
        self.current_goal_handle = None

        cbg = ReentrantCallbackGroup()

        # Transient Local QoS matching AMCL
        amcl_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            PoseWithCovarianceStamped, 
            f"/{self.robot_id}/amcl_pose", 
            self._on_pose, 
            amcl_qos, 
            callback_group=cbg
        )

        # Task Allocator Subscriptions & Publishers
        self.create_subscription(
            TaskAssignment,
            "/task_assignments",
            self._on_task_assignment,
            10,
            callback_group=cbg
        )
        self.create_subscription(
            Bool,
            "/mission_complete",
            self._on_mission_complete,
            10,
            callback_group=cbg
        )
        self.robot_state_pub = self.create_publisher(RobotState, "/robot_states", 10)

        # Nav2 Action Clients
        self.nav_client = ActionClient(self, NavigateToPose, f"/{self.robot_id}/navigate_to_pose", callback_group=cbg)
        self.spin_client = ActionClient(self, Spin, f"/{self.robot_id}/spin", callback_group=cbg)

        # Coverage Clients
        self.query_cli = self.create_client(QueryNearestUnseen, "query_nearest_unseen", callback_group=cbg)
        self.status_cli = self.create_client(GetRoomStatus, "get_room_status", callback_group=cbg)
        self.door_cli = self.create_client(ReserveDoor, "reserve_door", callback_group=cbg)

        for cli, name in [(self.query_cli, "query_nearest_unseen"),
                           (self.status_cli, "get_room_status"),
                           (self.door_cli, "reserve_door")]:
            while not cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f"[{self.robot_id}] waiting for service {name}...")

        self.get_logger().info(f"[{self.robot_id}] starting in room '{self.current_room}'")
        self.timer = self.create_timer(1.0, self._tick, callback_group=cbg)
        self.state_timer = self.create_timer(1.0, self._publish_robot_state, callback_group=cbg)

    def _publish_robot_state(self):
        """Periodically reports status to the Task Allocator."""
        if self.pose_x is None or self.pose_y is None:
            return

        msg = RobotState()
        msg.robot_id = self.robot_num_id
        msg.position = Point(x=float(self.pose_x), y=float(self.pose_y), z=0.0)
        msg.available = (self.mode == "SEARCH" and not self.finished)
        msg.current_target_id = self.current_target_id
        msg.battery_level = 100.0
        self.robot_state_pub.publish(msg)

    def _on_pose(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_pose_for_dist is not None:
            lx, ly = self._last_pose_for_dist
            self.dist_since_spin += math.hypot(x - lx, y - ly)
        self._last_pose_for_dist = (x, y)
        self.pose_x, self.pose_y = x, y

    def _on_mission_complete(self, msg: Bool):
        if msg.data:
            self.get_logger().info(f"[{self.robot_id}] Mission complete signal received!")
            if self.mode == "SEARCH":
                self.finished = True
                self._busy = False
            # If in COLLECT mode, it will finish its current collection task before stopping

    def _tick(self):
        if self.finished or self._busy or self.mode == "COLLECT":
            return
        
        if self.pose_x is None:
            if not self._warned_no_pose:
                self.get_logger().warn(f"[{self.robot_id}] Waiting for pose on /{self.robot_id}/amcl_pose...")
                self._warned_no_pose = True
            return

        self._busy = True
        self._step()

    # --- TASK ASSIGNMENT OVERRIDE ---
    def _on_task_assignment(self, msg: TaskAssignment):
        if msg.robot_id != self.robot_num_id:
            return

        self.get_logger().info(
            f"[{self.robot_id}] PREEMPTING SEARCH! Assigned to Target ID {msg.target_id} at ({msg.target_position.x:.2f}, {msg.target_position.y:.2f})"
        )

        # 1. Update State
        self.mode = "COLLECT"
        self.current_target_id = msg.target_id
        self._busy = True

        # 2. Cancel current search goal if active
        if self.current_goal_handle is not None:
            self.get_logger().info(f"[{self.robot_id}] Canceling current coverage goal...")
            self.current_goal_handle.cancel_goal_async()

        # 3. Report state change to allocator
        self._publish_robot_state()

        # 4. Command Nav2 to drive to the target position
        self._navigate_to(msg.target_position.x, msg.target_position.y)

    def _step(self):
        req = QueryNearestUnseen.Request()
        req.room_name = self.current_room
        req.robot_x, req.robot_y = self.pose_x, self.pose_y
        future = self.query_cli.call_async(req)
        future.add_done_callback(self._on_query_result)

    def _on_query_result(self, future):
        if self.mode == "COLLECT" or self.finished:
            return

        resp = future.result()
        if resp.found:
            self.get_logger().info(
                f"[{self.robot_id}] room '{self.current_room}': "
                f"{resp.unseen_count_in_room} cells unseen, heading to "
                f"({resp.target_x:.2f},{resp.target_y:.2f})"
            )
            self._navigate_to(resp.target_x, resp.target_y)
        else:
            self.get_logger().info(f"[{self.robot_id}] room '{self.current_room}' fully covered.")
            self._try_reallocate()

    def _navigate_to(self, x, y):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        if not self.nav_client.server_is_ready():
            self.get_logger().warn(f"[{self.robot_id}] nav server not ready yet, retrying next tick")
            self._busy = False
            return

        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_nav_goal_response)

    def _on_nav_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"[{self.robot_id}] nav goal rejected")
            self._busy = False
            if self.mode == "COLLECT":
                self.mode = "SEARCH"
                self.current_target_id = -1
            return
        
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        self.current_goal_handle = None

        if self.mode == "COLLECT":
            self.get_logger().info(f"[{self.robot_id}] REACHED TARGET {self.current_target_id}! Collection complete.")
            # Return to Search Mode
            self.mode = "SEARCH"
            self.current_target_id = -1
            self._busy = False
            self._publish_robot_state()
            return

        # Coverage behavior: spin check
        if self.dist_since_spin >= self.spin_interval_m:
            self._do_spin_then_continue()
        else:
            self._busy = False

    def _do_spin_then_continue(self):
        self.dist_since_spin = 0.0
        goal = Spin.Goal()
        goal.target_yaw = 2.0 * math.pi
        if not self.spin_client.server_is_ready():
            self.get_logger().warn(f"[{self.robot_id}] spin server not ready, skipping")
            self._busy = False
            return
        send_future = self.spin_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_spin_goal_response)

    def _on_spin_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._busy = False
            return
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: setattr(self, "_busy", False))

    def _try_reallocate(self):
        req = GetRoomStatus.Request()
        req.room_name = ""
        future = self.status_cli.call_async(req)
        future.add_done_callback(self._on_status_for_reallocation)

    def _on_status_for_reallocation(self, future):
        resp = future.result()
        candidates = [(name, cnt) for name, cnt in zip(resp.room_names, resp.unseen_counts)
                      if name != self.current_room and cnt > 0]
        if not candidates:
            self.get_logger().info(f"[{self.robot_id}] no work left anywhere. Done.")
            self.finished = True
            self._busy = False
            return

        target_room, _ = max(candidates, key=lambda kv: kv[1])
        door = self._find_door(self.current_room, target_room)
        if door is None:
            self.get_logger().warn(
                f"[{self.robot_id}] no direct door from {self.current_room} to {target_room}, treating as done."
            )
            self.finished = True
            self._busy = False
            return

        self.get_logger().info(f"[{self.robot_id}] reallocating {self.current_room} -> {target_room} via door '{door['name']}'")
        self._reserve_and_cross(door, target_room)

    def _find_door(self, room_a, room_b):
        for d in self.doors:
            if set(d["connects"]) == {room_a, room_b}:
                return d
        return None

    def _reserve_and_cross(self, door, target_room):
        req = ReserveDoor.Request()
        req.door_name = door["name"]
        req.robot_id = self.robot_id
        req.release = False
        future = self.door_cli.call_async(req)
        future.add_done_callback(lambda f: self._on_door_reserved(f, door, target_room))

    def _on_door_reserved(self, future, door, target_room):
        resp = future.result()
        if not resp.granted:
            self.get_logger().info(f"[{self.robot_id}] door '{door['name']}' busy, retrying shortly")
            self._busy = False
            return

        cx, cy = door["center"]
        self._navigate_to(cx, cy)
        self.current_room = target_room
        rel = ReserveDoor.Request()
        rel.door_name = door["name"]
        rel.robot_id = self.robot_id
        rel.release = True
        self.door_cli.call_async(rel)


def main(args=None):
    rclpy.init(args=args)
    node = RobotCoverageNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
