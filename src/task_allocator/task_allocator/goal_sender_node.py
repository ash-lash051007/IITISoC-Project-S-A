#!/usr/bin/env python3
"""
goal_sender_node.py

Runs ONE INSTANCE PER ROBOT (namespaced), inside the task_allocator package.

Job:
  1. Caches all known target positions from the GLOBAL /waste_database/targets
     topic (target_id -> position, waste_type).
  2. Tracks THIS robot's current position from the GLOBAL
     /waste_database/robot_states topic (filtered by robot_name) -- sourced
     from the robot's odom bridge, NOT from AMCL, since AMCL was found to
     never actually publish in this setup (topic exists, no live publisher).
  3. Listens on the GLOBAL /task_allocator/assignments topic and filters for
     assignments addressed to THIS robot (by robot_name).
  4. On a new assignment, looks up the target's position from the cache
     built in step 1 (no service call needed) and sends it as a Nav2
     NavigateToPose goal to THIS robot's own namespaced Nav2 stack.
  5. Continuously monitors this robot's live position (from step 2) against
     the target position. As soon as the robot comes within
     COLLECTION_RADIUS_M (1.0 m) of the target, it is treated as
     "collected": the Nav2 goal is cancelled (robot simply stops where it
     is), and /waste_database/report_task_status is called with
     status='COMPLETED'.
  6. Goes idle again, waiting for the next assignment.

Launch one instance per robot, e.g.:
  ros2 run task_allocator goal_sender_node --ros-args \
      -r __ns:=/robot1 -p robot_name:=robot1
  ros2 run task_allocator goal_sender_node --ros-args \
      -r __ns:=/robot2 -p robot_name:=robot2
  ros2 run task_allocator goal_sender_node --ros-args \
      -r __ns:=/robot3 -p robot_name:=robot3

NOTE ON TOPIC/SERVICE SCOPE:
  /waste_database/targets, /waste_database/robot_states,
  /task_allocator/assignments, and /waste_database/report_task_status are
  all GLOBAL (absolute, leading-slash) names -- they resolve the same
  regardless of which robot namespace this node instance is launched
  under. Only navigate_to_pose is left RELATIVE, so ROS2 namespacing
  correctly scopes it to this robot's own Nav2 stack alone
  (e.g. /robot2/navigate_to_pose).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from waste_interfaces.msg import TaskAssignmentArray, WasteTargetArray, RobotStateArray
from waste_interfaces.srv import ReportTaskStatus


COLLECTION_RADIUS_M = 1.0          # within this distance of the target = collected
DISTANCE_CHECK_PERIOD_SEC = 0.5    # how often to check distance-to-target


class GoalSenderNode(Node):
    def __init__(self):
        super().__init__('goal_sender_node')

        self.declare_parameter('robot_name', '')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        if not self.robot_name:
            self.get_logger().error(
                'robot_name parameter not set -- pass -p robot_name:=robotN when '
                'launching. This node cannot function without it.')

        # --- internal state ---
        self.current_pose = None          # geometry_msgs/Pose, from /waste_database/robot_states
        self.targets_cache = {}           # target_id -> {'position': Point, 'waste_type': str}
        self.pending_assignment = None    # (task_id, target_id) waiting on a target_id not yet cached
        self.active_task_id = None
        self.active_target_id = None
        self.active_target_position = None
        self.goal_handle = None

        # --- QoS matching waste_database_node's / task_allocator_node's
        # transient_local publishers, so a late-starting goal_sender still
        # receives the most recent state instead of missing it. ---
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # --- GLOBAL topics ---
        self.create_subscription(
            WasteTargetArray, '/waste_database/targets',
            self.on_targets_array, latched_qos)
        self.create_subscription(
            RobotStateArray, '/waste_database/robot_states',
            self.on_robot_states_array, latched_qos)
        self.create_subscription(
            TaskAssignmentArray, '/task_allocator/assignments',
            self.on_assignment_array, latched_qos)

        # --- GLOBAL service: report collection ---
        self.report_status_client = self.create_client(
            ReportTaskStatus, '/waste_database/report_task_status')

        # --- namespaced (relative): this robot's own Nav2 action server ---
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- periodic check: have we reached within COLLECTION_RADIUS_M? ---
        self.create_timer(DISTANCE_CHECK_PERIOD_SEC, self.check_collection)

        # --- periodic retry: if we have an active task but no live Nav2
        # goal (e.g. the first send failed because Nav2 wasn't fully up
        # yet), keep retrying instead of leaving the task stuck forever. ---
        self.create_timer(3.0, self.retry_goal_if_needed)

        self.get_logger().info(f'goal_sender_node started for robot_name={self.robot_name}')

    # ------------------------------------------------------------------
    # Caches
    # ------------------------------------------------------------------
    def on_targets_array(self, msg: WasteTargetArray):
        for wt in msg.targets:
            self.targets_cache[wt.target_id] = {
                'position': wt.position,
                'waste_type': wt.waste_type,
            }

        # A new targets broadcast may resolve an assignment we couldn't
        # act on earlier because its target wasn't cached yet.
        if self.pending_assignment is not None:
            task_id, target_id = self.pending_assignment
            if target_id in self.targets_cache:
                self.pending_assignment = None
                self._start_task(task_id, target_id)

    def on_robot_states_array(self, msg: RobotStateArray):
        for rs in msg.robots:
            if rs.robot_name == self.robot_name:
                self.current_pose = rs.pose
                return

    # ------------------------------------------------------------------
    # Assignment handling
    # ------------------------------------------------------------------
    def on_assignment_array(self, msg: TaskAssignmentArray):
        for ta in msg.tasks:
            if ta.robot_name != self.robot_name:
                continue
            if ta.task_id == self.active_task_id:
                continue  # already handling this one, ignore repeat broadcasts
            self._start_task(ta.task_id, ta.target_id)

    def _start_task(self, task_id, target_id):
        cached = self.targets_cache.get(target_id)
        if cached is None:
            self.get_logger().warn(
                f'Assignment for task {task_id} (target {target_id}) arrived before '
                f'its target position was cached -- deferring until /waste_database/'
                f'targets includes it.')
            self.pending_assignment = (task_id, target_id)
            return

        self.active_task_id = task_id
        self.active_target_id = target_id
        self.active_target_position = cached['position']

        self.get_logger().info(
            f'Heading to task {task_id}, target {target_id} at '
            f'({cached["position"].x:.2f}, {cached["position"].y:.2f})')

        self._send_nav_goal(cached['position'])

    # ------------------------------------------------------------------
    def _send_nav_goal(self, position):
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                'Nav2 action server not available in this namespace yet -- '
                'will keep retrying every 3s until it comes up.')
            return  # active_task_id stays set; retry_goal_if_needed() will try again

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = position
        goal.pose.pose.orientation.w = 1.0  # no specific final heading required

        send_future = self._nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Nav2 rejected the goal')
            return
        self.goal_handle = handle
        # We do NOT wait on this to decide "collected" -- that's decided by
        # check_collection() below. This callback exists so that IF Nav2
        # reaches its own exact goal tolerance before our radius check
        # fires, we still mark the task complete rather than sit idle.
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        if self.active_task_id is not None:
            self.get_logger().info('Nav2 reports goal reached')
            self._mark_collected()

    # ------------------------------------------------------------------
    def retry_goal_if_needed(self):
        # If we have an active task, but no live goal_handle, the previous
        # send attempt either never happened or failed (e.g. Nav2 action
        # server wasn't up yet) -- try again.
        if self.active_task_id is not None and self.goal_handle is None:
            if self.active_target_position is not None:
                self.get_logger().info(
                    f'Retrying goal send for task {self.active_task_id} '
                    f'(target {self.active_target_id})')
                self._send_nav_goal(self.active_target_position)

    # ------------------------------------------------------------------
    def check_collection(self):
        if (self.active_task_id is None
                or self.current_pose is None
                or self.active_target_position is None):
            return

        dx = self.active_target_position.x - self.current_pose.position.x
        dy = self.active_target_position.y - self.current_pose.position.y
        dist = math.hypot(dx, dy)

        if dist <= COLLECTION_RADIUS_M:
            self.get_logger().info(
                f'Within {dist:.2f}m of target {self.active_target_id} '
                f'(<= {COLLECTION_RADIUS_M}m) -- treating as collected, stopping here')
            self._cancel_nav_goal()
            self._mark_collected()

    def _cancel_nav_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    # ------------------------------------------------------------------
    def _mark_collected(self):
        if self.active_task_id is None:
            return  # already handled (radius check and Nav2 result both fired)

        task_id = self.active_task_id
        self.active_task_id = None
        self.active_target_id = None
        self.active_target_position = None

        if not self.report_status_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                f'/waste_database/report_task_status not available -- could not '
                f'report task {task_id} as COMPLETED')
            return

        req = ReportTaskStatus.Request()
        req.task_id = task_id
        req.robot_name = self.robot_name
        req.status = 'COMPLETED'

        future = self.report_status_client.call_async(req)
        future.add_done_callback(self._on_report_status_response)

    def _on_report_status_response(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'report_task_status call failed: {e}')
            return
        if resp.success:
            self.get_logger().info(f'Task reported COMPLETED: {resp.message}')
        else:
            self.get_logger().error(f'report_task_status rejected: {resp.message}')


def main(args=None):
    rclpy.init(args=args)
    node = GoalSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
