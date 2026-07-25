#!/usr/bin/env python3
"""
task_allocator_node.py

Centralized dynamic task allocator for the multi-robot waste-collection
framework. Runs inside the shared "database" node's process (or as a
sibling node talking to it) and computes globally-optimal robot-target
assignments using the Hungarian algorithm (scipy.optimize.linear_sum_assignment).

Design decisions (see project write-up for justification):
  - Event-triggered recomputation only (new target / task completion /
    robot state change) -> avoids assignment thrashing from recomputing
    every tick.
  - Targets are LOCKED the instant they are assigned, so a re-trigger
    mid-execution cannot reassign a target that's already being worked.
  - Cost matrix combines: travel distance, target priority, and robot
    current workload. Weights are tunable constants below.
  - A GREEDY mode is included behind a parameter so you can run both
    strategies on the same target set and log the comparison for the
    "evaluate allocation efficiency" bonus requirement.
"""

import rclpy
from rclpy.node import Node
from scipy.optimize import linear_sum_assignment
import numpy as np

from task_allocator_msgs.msg import Target, RobotState, TaskAssignment


# ---- tunable cost weights ----
W_DISTANCE = 1.0
W_PRIORITY = -2.0     # negative: higher priority target -> lower cost -> preferred
W_WORKLOAD = 0.5       # penalty per already-queued task for that robot


class TaskAllocatorNode(Node):
    def __init__(self):
        super().__init__('task_allocator_node')

        self.declare_parameter('strategy', 'hungarian')  # 'hungarian' or 'greedy'
        self.strategy = self.get_parameter('strategy').get_parameter_value().string_value

        self.targets = {}       # target_id -> Target
        self.robots = {}        # robot_id -> RobotState
        self.robot_workload = {}  # robot_id -> int, in-flight task count

        self.create_subscription(Target, '/waste_targets', self.on_target_update, 10)
        self.create_subscription(RobotState, '/robot_states', self.on_robot_update, 10)

        self.assignment_pub = self.create_publisher(TaskAssignment, '/task_assignments', 10)

        self.get_logger().info(f'Task allocator started, strategy={self.strategy}')

    # ------------------------------------------------------------------
    # Event handlers -> trigger recompute
    # ------------------------------------------------------------------
    def on_target_update(self, msg: Target):
        existing = self.targets.get(msg.target_id)
        # Ignore duplicate detections of an already-known target
        if existing is not None:
            return
        self.targets[msg.target_id] = msg
        self.get_logger().info(f'New target {msg.target_id} at '
                                f'({msg.position.x:.2f}, {msg.position.y:.2f})')
        self.recompute_assignments()

    def on_robot_update(self, msg: RobotState):
        prev = self.robots.get(msg.robot_id)
        self.robots[msg.robot_id] = msg

        # A robot going idle (finished/aborted its task) frees the target
        if prev is not None and prev.current_target_id != -1 and msg.current_target_id == -1:
            completed_id = prev.current_target_id
            if completed_id in self.targets:
                self.targets[completed_id].status = 'COMPLETED'
                self.robot_workload[msg.robot_id] = max(
                    0, self.robot_workload.get(msg.robot_id, 1) - 1)
                self.get_logger().info(f'Target {completed_id} marked COMPLETED '
                                        f'(finished by robot {msg.robot_id})')
            self.recompute_assignments()

    # ------------------------------------------------------------------
    # Allocation
    # ------------------------------------------------------------------
    def recompute_assignments(self):
        available_robots = [r for r in self.robots.values() if r.available]
        open_targets = [t for t in self.targets.values() if t.status == 'UNASSIGNED']

        if not available_robots or not open_targets:
            return

        if self.strategy == 'greedy':
            assignments = self._allocate_greedy(available_robots, open_targets)
        else:
            assignments = self._allocate_hungarian(available_robots, open_targets)

        for robot_id, target_id in assignments:
            self._lock_and_publish(robot_id, target_id)

    def _build_cost_matrix(self, robots, targets):
        n_r, n_t = len(robots), len(targets)
        size = max(n_r, n_t)
        # Pad with zero-cost dummy rows/cols so the matrix is square,
        # required by linear_sum_assignment for a clean 1:1 pairing.
        cost = np.zeros((size, size))
        for i, r in enumerate(robots):
            for j, t in enumerate(targets):
                dist = np.hypot(t.position.x - r.position.x,
                                 t.position.y - r.position.y)
                workload = self.robot_workload.get(r.robot_id, 0)
                cost[i, j] = (W_DISTANCE * dist
                              + W_PRIORITY * t.priority
                              + W_WORKLOAD * workload)
        return cost

    def _allocate_hungarian(self, robots, targets):
        cost = self._build_cost_matrix(robots, targets)
        row_ind, col_ind = linear_sum_assignment(cost)

        assignments = []
        for r_idx, t_idx in zip(row_ind, col_ind):
            if r_idx < len(robots) and t_idx < len(targets):
                assignments.append((robots[r_idx].robot_id, targets[t_idx].target_id))
        return assignments

    def _allocate_greedy(self, robots, targets):
        """Nearest-target-first, used only as a baseline for comparison logging."""
        assignments = []
        remaining_robots = list(robots)
        remaining_targets = list(targets)
        while remaining_robots and remaining_targets:
            best = None
            for r in remaining_robots:
                for t in remaining_targets:
                    d = np.hypot(t.position.x - r.position.x, t.position.y - r.position.y)
                    if best is None or d < best[0]:
                        best = (d, r, t)
            _, r, t = best
            assignments.append((r.robot_id, t.target_id))
            remaining_robots.remove(r)
            remaining_targets.remove(t)
        return assignments

    def _lock_and_publish(self, robot_id, target_id):
        self.targets[target_id].status = 'LOCKED'
        self.targets[target_id].assigned_robot_id = robot_id
        self.robot_workload[robot_id] = self.robot_workload.get(robot_id, 0) + 1

        msg = TaskAssignment()
        msg.robot_id = robot_id
        msg.target_id = target_id
        msg.target_position = self.targets[target_id].position
        self.assignment_pub.publish(msg)

        self.get_logger().info(f'Assigned robot {robot_id} -> target {target_id}')


def main(args=None):
    rclpy.init(args=args)
    node = TaskAllocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
