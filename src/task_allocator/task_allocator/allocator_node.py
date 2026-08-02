#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from scipy.optimize import linear_sum_assignment
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import numpy as np

from waste_interfaces.msg import WasteTargetArray, RobotStateArray, TaskAssignmentArray, TaskAssignment

W_DISTANCE = 1.0
W_WORKLOAD = 0.5

class TaskAllocatorNode(Node):
    def __init__(self):
        super().__init__('task_allocator_node')
        self.declare_parameter('strategy', 'hungarian')
        self.strategy = self.get_parameter('strategy').get_parameter_value().string_value

        self.targets = {}
        self.robots = {}
        self.robot_workload = {}

        self.create_subscription(WasteTargetArray, '/waste_database/targets', self.on_target_array_update, 10)
        self.create_subscription(RobotStateArray, '/waste_database/robot_states', self.on_robot_array_update, 10)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.assignment_pub = self.create_publisher(TaskAssignmentArray, '/task_allocator/assignments', qos)

        self.get_logger().info(f'Task allocator started, strategy={self.strategy}, ready for ARRAY updates')

    def on_target_array_update(self, msg: WasteTargetArray):
        changed = False
        for target in msg.targets: 
            if target.target_id not in self.targets:
                self.targets[target.target_id] = target
                self.get_logger().info(f'New target {target.target_id} detected at ({target.position.x:.2f}, {target.position.y:.2f})')
                changed = True
        
        if changed:
            self.recompute_assignments()

    def on_robot_array_update(self, msg: RobotStateArray):
        changed = False
        for robot in msg.robots:
            prev = self.robots.get(robot.robot_id)
            self.robots[robot.robot_id] = robot

            if prev is not None and prev.current_task_id != -1 and robot.current_task_id == -1:
                completed_id = prev.current_task_id
                if completed_id in self.targets:
                    self.targets[completed_id].status = 'COLLECTED'
                    self.targets[completed_id].collected_by = robot.robot_name
                    self.robot_workload[robot.robot_id] = max(0, self.robot_workload.get(robot.robot_id, 1) - 1)
                    self.get_logger().info(f'Target {completed_id} finished by {robot.robot_name}')
                    changed = True
            
            elif prev is None and robot.status == 'IDLE':
                changed = True

        if changed:
            self.recompute_assignments()

    def recompute_assignments(self):
        available_robots = [r for r in self.robots.values() if r.status == 'IDLE']
        open_targets = [t for t in self.targets.values() if t.status == 'PENDING']

        if not available_robots or not open_targets:
            return

        if self.strategy == 'greedy':
            assignments = self._allocate_greedy(available_robots, open_targets)
        else:
            assignments = self._allocate_hungarian(available_robots, open_targets)

        if not assignments:
            return

        assignment_array_msg = TaskAssignmentArray()

        for robot_id, target_id in assignments:
            robot_name = self.robots[robot_id].robot_name
            
            self.targets[target_id].status = 'ASSIGNED'
            self.targets[target_id].assigned_robot = robot_name
            self.robot_workload[robot_id] = self.robot_workload.get(robot_id, 0) + 1

            # Perfectly mapped to your TaskAssignment definition
            single_assignment = TaskAssignment()
            single_assignment.task_id = target_id
            single_assignment.target_id = target_id
            single_assignment.robot_name = robot_name
            single_assignment.state = 'ASSIGNED'
            # (Leaving assigned_time at default 0 to keep it simple and avoid clock syncing issues)
            
            assignment_array_msg.tasks.append(single_assignment)
            
            self.get_logger().info(f'*** MATCH FOUND: {robot_name} -> Target {target_id} ***')

        self.assignment_pub.publish(assignment_array_msg)

    def _build_cost_matrix(self, robots, targets):
        n_r, n_t = len(robots), len(targets)
        size = max(n_r, n_t)
        cost = np.zeros((size, size))
        for i, r in enumerate(robots):
            for j, t in enumerate(targets):
                dist = np.hypot(t.position.x - r.pose.position.x, t.position.y - r.pose.position.y)
                workload = self.robot_workload.get(r.robot_id, 0)
                cost[i, j] = (W_DISTANCE * dist + W_WORKLOAD * workload)
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
        assignments = []
        remaining_robots = list(robots)
        remaining_targets = list(targets)
        while remaining_robots and remaining_targets:
            best = None
            for r in remaining_robots:
                for t in remaining_targets:
                    d = np.hypot(t.position.x - r.pose.position.x, t.position.y - r.pose.position.y)
                    if best is None or d < best[0]:
                        best = (d, r, t)
            _, r, t = best
            assignments.append((r.robot_id, t.target_id))
            remaining_robots.remove(r)
            remaining_targets.remove(t)
        return assignments

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
