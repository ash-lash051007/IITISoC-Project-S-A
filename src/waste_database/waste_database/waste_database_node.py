#!/usr/bin/env python3
"""
waste_database_node.py

Centralized shared ROS2 node for the multi-robot waste collection framework.

Responsibilities (per project spec):
  - Maintain detected waste targets, robot states, and active task assignments.
  - Prevent duplicate detections of the same waste target.
  - Prevent redundant task assignments (two robots getting the same target).
  - Keep environment info synchronized across agents via periodic broadcast topics.
  - Expose clearly defined services for robot/perception <-> database communication.

Design note on duplicate prevention:
  Each ArUco marker ID (0-11 in this project) uniquely identifies one physical
  waste target. We use that marker ID directly as the target_id in the
  database. This makes duplicate detection an exact key lookup rather than a
  fuzzy distance comparison -- if the ID has been seen before, it is the same
  target, and we simply refine its stored position instead of creating a
  second entry.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Point, Pose

from waste_interfaces.msg import (
    WasteTarget,
    WasteTargetArray,
    RobotState,
    RobotStateArray,
    TaskAssignment,
    TaskAssignmentArray,
)
from waste_interfaces.srv import RegisterDetection, RequestTask, ReportTaskStatus


VALID_STATUSES = {'IN_PROGRESS', 'COMPLETED', 'FAILED'}


class C:
    """ANSI color codes for terminal log readability. Purely cosmetic --
    never used inside actual service response fields, only in what gets
    printed to this node's own terminal."""
    RESET = '\033[0m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    CYAN = '\033[96m'
    RED = '\033[91m'
    BOLD_GREEN = '\033[1;92m'


class WasteDatabaseNode(Node):

    def __init__(self):
        super().__init__('waste_database_node')

        # --- in-memory state ---------------------------------------------
        # Guarded by self.lock since service callbacks and the publish timer
        # can be invoked from different callback groups/threads depending on
        # the executor used.
        self.lock = threading.Lock()

        self.targets = {}        # target_id (int)  -> dict
        self.robots = {}         # robot_name (str) -> dict
        self.tasks = {}          # task_id (int)    -> dict
        self._next_task_id = 1

        # --- parameters -----------------------------------------------------
        self.declare_parameter('publish_rate_hz', 2.0)
        publish_rate = self.get_parameter('publish_rate_hz').value

        # --- QoS: transient_local so late-joining subscribers (RViz, a
        # dashboard, a robot that (re)starts late) immediately get the
        # latest known state instead of waiting for the next timer tick. ---
        latched_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # --- publishers: broadcast synchronized environment state --------
        self.targets_pub = self.create_publisher(
            WasteTargetArray, '/waste_database/targets', latched_qos)
        self.robots_pub = self.create_publisher(
            RobotStateArray, '/waste_database/robot_states', latched_qos)
        self.tasks_pub = self.create_publisher(
            TaskAssignmentArray, '/waste_database/task_assignments', latched_qos)

        # --- subscriber: robots report their own telemetry here ----------
        self.create_subscription(
            RobotState, '/robot_state', self.robot_state_callback, 10)

        # --- services: robot/perception <-> database communication -------
        self.create_service(
            RegisterDetection, '/waste_database/register_detection',
            self.handle_register_detection)
        self.create_service(
            RequestTask, '/waste_database/request_task',
            self.handle_request_task)
        self.create_service(
            ReportTaskStatus, '/waste_database/report_task_status',
            self.handle_report_task_status)

        # --- periodic broadcast timer -------------------------------------
        self.create_timer(1.0 / publish_rate, self.publish_state)

        self.get_logger().info('waste_database_node is up.')
        self.get_logger().info('Services: /waste_database/register_detection, '
                                '/waste_database/request_task, '
                                '/waste_database/report_task_status')
        self.get_logger().info('Topics out: /waste_database/targets, '
                                '/waste_database/robot_states, '
                                '/waste_database/task_assignments')
        self.get_logger().info('Topic in : /robot_state')

    # ----------------------------------------------------------------- #
    # Service: RegisterDetection
    # ----------------------------------------------------------------- #
    def handle_register_detection(self, request, response):
        with self.lock:
            marker_id = request.marker_id
            now = self.get_clock().now().to_msg()

            if marker_id in self.targets:
                # Already known -> this is a duplicate detection of the same
                # physical target. Refine its stored position with a
                # confidence-weighted average instead of adding a new entry.
                target = self.targets[marker_id]
                old_conf = target['confidence']
                new_conf = request.confidence
                total = old_conf + new_conf
                w_old, w_new = (old_conf / total, new_conf / total) if total > 0 else (0.5, 0.5)

                target['position'].x = w_old * target['position'].x + w_new * request.position.x
                target['position'].y = w_old * target['position'].y + w_new * request.position.y
                target['position'].z = w_old * target['position'].z + w_new * request.position.z
                target['confidence'] = max(old_conf, new_conf)
                target['last_seen'] = now
                target['last_detected_by'] = request.robot_name
                target['detection_count'] += 1

                response.accepted = True
                response.is_duplicate = True
                response.target_id = marker_id
                response.message = (
                    f'Target {marker_id} already known (seen '
                    f'{target["detection_count"]}x) - position refined, no new entry created.'
                )
                self.get_logger().info(f'{C.YELLOW}{response.message}{C.RESET}')
                self.targets[marker_id] = {
                    'position': Point(x=request.position.x,
                                       y=request.position.y,
                                       z=request.position.z),
                    'waste_type': request.waste_type,
                    'status': 'PENDING',
                    'confidence': request.confidence,
                    'first_detected_by': request.robot_name,
                    'last_detected_by': request.robot_name,
                    'detection_count': 1,
                    'assigned_robot': '',
                    'collected_by': '',
                    'first_seen': now,
                    'last_seen': now,
                }
                response.accepted = True
                response.is_duplicate = False
                response.target_id = marker_id
                response.message = f'New target {marker_id} registered by {request.robot_name}.'
                self.get_logger().info(f'{C.GREEN}{response.message}{C.RESET}')

        return response

    # ----------------------------------------------------------------- #
    # Service: RequestTask
    # ----------------------------------------------------------------- #
    def handle_request_task(self, request, response):
        with self.lock:
            best_id = None
            best_dist = float('inf')
            rp = request.robot_pose.position

            for tid, t in self.targets.items():
                if t['status'] != 'PENDING':
                    # Already ASSIGNED or COLLECTED -> not up for grabs.
                    # This is what prevents redundant/duplicate assignment.
                    continue
                dist = math.hypot(t['position'].x - rp.x, t['position'].y - rp.y)
                if dist < best_dist:
                    best_dist = dist
                    best_id = tid

            if best_id is None:
                response.task_available = False
                response.task_id = -1
                response.target_id = -1
                self.get_logger().info(
                    f'{C.YELLOW}{request.robot_name} requested a task but none are '
                    f'available (no PENDING targets).{C.RESET}')
                return response

            task_id = self._next_task_id
            self._next_task_id += 1

            self.tasks[task_id] = {
                'target_id': best_id,
                'robot_name': request.robot_name,
                'state': 'ASSIGNED',
                'assigned_time': self.get_clock().now().to_msg(),
            }
            self.targets[best_id]['status'] = 'ASSIGNED'
            self.targets[best_id]['assigned_robot'] = request.robot_name

            if request.robot_name in self.robots:
                self.robots[request.robot_name]['current_task_id'] = task_id
                self.robots[request.robot_name]['status'] = 'ASSIGNED'

            response.task_available = True
            response.task_id = task_id
            response.target_id = best_id
            response.target_position = self.targets[best_id]['position']
            response.waste_type = self.targets[best_id]['waste_type']

            self.get_logger().info(
                f'{C.CYAN}Assigned task {task_id} (target {best_id}, dist={best_dist:.2f}m) '
                f'to {request.robot_name}.{C.RESET}')

        return response

    # ----------------------------------------------------------------- #
    # Service: ReportTaskStatus
    # ----------------------------------------------------------------- #
    def handle_report_task_status(self, request, response):
        with self.lock:
            task = self.tasks.get(request.task_id)
            status = request.status.upper()

            if task is None:
                response.success = False
                response.message = f'No such task {request.task_id}.'
                self.get_logger().warn(f'{C.RED}{response.message}{C.RESET}')
                return response

            if task['robot_name'] != request.robot_name:
                response.success = False
                response.message = (
                    f'Task {request.task_id} belongs to {task["robot_name"]}, '
                    f'not {request.robot_name}.'
                )
                self.get_logger().warn(f'{C.RED}{response.message}{C.RESET}')
                return response

            if status not in VALID_STATUSES:
                response.success = False
                response.message = f'Unknown status "{request.status}".'
                self.get_logger().warn(f'{C.RED}{response.message}{C.RESET}')
                return response

            task['state'] = status
            target_id = task['target_id']

            if status == 'COMPLETED':
                self.targets[target_id]['status'] = 'COLLECTED'
                self.targets[target_id]['collected_by'] = request.robot_name
                self.targets[target_id]['assigned_robot'] = ''
                self._free_robot(request.robot_name)
            elif status == 'FAILED':
                # Reopen the target so it can be picked up again (by this
                # robot or the other one) instead of being stuck forever.
                self.targets[target_id]['status'] = 'PENDING'
                self.targets[target_id]['assigned_robot'] = ''
                self._free_robot(request.robot_name)
            # IN_PROGRESS: just update state, nothing else changes.

            response.success = True
            response.message = f'Task {request.task_id} updated to {status}.'
            log_color = C.BOLD_GREEN if status == 'COMPLETED' else (
                C.RED if status == 'FAILED' else C.CYAN)
            self.get_logger().info(f'{log_color}{response.message}{C.RESET}')

        return response

    def _free_robot(self, robot_name):
        if robot_name in self.robots:
            self.robots[robot_name]['current_task_id'] = -1
            self.robots[robot_name]['status'] = 'IDLE'

    # ----------------------------------------------------------------- #
    # Subscriber: /robot_state
    # ----------------------------------------------------------------- #
    def robot_state_callback(self, msg):
        with self.lock:
            entry = self.robots.setdefault(msg.robot_name, {})
            entry.update({
                'robot_id': msg.robot_id,
                'pose': msg.pose,
                'status': msg.status,
                'current_task_id': msg.current_task_id,
                'battery_level': msg.battery_level,
                'last_update': self.get_clock().now().to_msg(),
            })

    # ----------------------------------------------------------------- #
    # Periodic broadcast: keeps all agents synchronized
    # ----------------------------------------------------------------- #
    def publish_state(self):
        with self.lock:
            target_array = WasteTargetArray()
            for tid, t in self.targets.items():
                wt = WasteTarget()
                wt.target_id = tid
                wt.position = t['position']
                wt.waste_type = t['waste_type']
                wt.status = t['status']
                wt.confidence = t['confidence']
                wt.first_detected_by = t['first_detected_by']
                wt.last_detected_by = t['last_detected_by']
                wt.detection_count = t['detection_count']
                wt.assigned_robot = t['assigned_robot']
                wt.collected_by = t['collected_by']
                wt.first_seen = t['first_seen']
                wt.last_seen = t['last_seen']
                target_array.targets.append(wt)
            self.targets_pub.publish(target_array)

            robot_array = RobotStateArray()
            for name, r in self.robots.items():
                rs = RobotState()
                rs.robot_name = name
                rs.robot_id = r.get('robot_id', 0)
                rs.pose = r.get('pose', Pose())
                rs.status = r.get('status', 'UNKNOWN')
                rs.current_task_id = r.get('current_task_id', -1)
                rs.battery_level = r.get('battery_level', 100.0)
                rs.last_update = r.get('last_update', self.get_clock().now().to_msg())
                robot_array.robots.append(rs)
            self.robots_pub.publish(robot_array)

            task_array = TaskAssignmentArray()
            for task_id, t in self.tasks.items():
                ta = TaskAssignment()
                ta.task_id = task_id
                ta.target_id = t['target_id']
                ta.robot_name = t['robot_name']
                ta.state = t['state']
                ta.assigned_time = t['assigned_time']
                task_array.tasks.append(ta)
            self.tasks_pub.publish(task_array)


def main(args=None):
    rclpy.init(args=args)
    node = WasteDatabaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
