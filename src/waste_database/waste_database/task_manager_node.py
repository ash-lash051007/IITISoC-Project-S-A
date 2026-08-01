#!/usr/bin/env python3
"""
task_manager_node.py

Runs the full task lifecycle for ONE robot:
  1. When idle, ask the shared database for a task (request_task).
  2. If one is available, send it to Nav2 as a NavigateToPose goal.
  3. When Nav2 finishes (success or failure), report the outcome back to
     the database (report_task_status).
  4. Repeat.

One instance runs per robot (see task_managers.launch.py), each given its
own robot_name / odom_topic / navigate_action via parameters.

This node owns the "what task am I doing" truth for its robot. It publishes
that on a small local topic (<robot_name>/current_task_id, std_msgs/Int32)
rather than writing directly into the shared /robot_state broadcast --
robot_state_bridge_node listens to this and relays it, so there's only ever
one place that decides what a robot's current_task_id actually is.

Uses a multi-threaded executor: this node has to wait on Nav2 (which can
take a while per goal) while still staying responsive to odometry updates
and the next task-request timer tick, so a single-threaded spin would let
a long navigation goal block everything else.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from waste_interfaces.srv import RequestTask, ReportTaskStatus

NAV2_STATUS_SUCCEEDED = 4


class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('task_manager_node')

        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('odom_topic', '/robot1/odom')
        self.declare_parameter('navigate_action', '/robot1/navigate_to_pose')
        self.declare_parameter('retry_period_sec', 3.0)

        self.robot_name = self.get_parameter('robot_name').value
        odom_topic = self.get_parameter('odom_topic').value
        nav_action_name = self.get_parameter('navigate_action').value
        retry_period = self.get_parameter('retry_period_sec').value

        self.latest_pose = None
        self.busy = False
        self.current_task_id = -1
        self.current_target_id = -1

        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10, callback_group=cb_group)

        self.request_task_client = self.create_client(
            RequestTask, '/waste_database/request_task', callback_group=cb_group)
        self.report_status_client = self.create_client(
            ReportTaskStatus, '/waste_database/report_task_status', callback_group=cb_group)
        self.nav_client = ActionClient(
            self, NavigateToPose, nav_action_name, callback_group=cb_group)

        self.task_id_pub = self.create_publisher(Int32, f'{self.robot_name}/current_task_id', 10)

        self.create_timer(retry_period, self.try_request_task, callback_group=cb_group)

        self.get_logger().info(
            f'task_manager_node up for "{self.robot_name}", nav action={nav_action_name}')

    def odom_callback(self, msg: Odometry):
        self.latest_pose = msg.pose.pose

    def try_request_task(self):
        if self.busy or self.latest_pose is None:
            return
        if not self.request_task_client.service_is_ready():
            self.get_logger().warn(
                'waste_database/request_task not available yet.', throttle_duration_sec=10.0)
            return

        req = RequestTask.Request()
        req.robot_name = self.robot_name
        req.robot_pose.position = self.latest_pose.position
        req.robot_pose.orientation = self.latest_pose.orientation

        self.request_task_client.call_async(req).add_done_callback(self.on_task_response)

    def on_task_response(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f'request_task call failed: {e}')
            return

        if not resp.task_available:
            return  # nothing pending right now -- next timer tick will try again

        self.busy = True
        self.current_task_id = resp.task_id
        self.current_target_id = resp.target_id
        self.task_id_pub.publish(Int32(data=self.current_task_id))

        self.get_logger().info(
            f'{self.robot_name}: got task {resp.task_id} -> target {resp.target_id} '
            f'({resp.waste_type}) at ({resp.target_position.x:.2f}, {resp.target_position.y:.2f})'
        )
        self.send_nav_goal(resp.target_position)

    def send_nav_goal(self, position):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{self.robot_name}: Nav2 action server not available, failing task.')
            self.finish_task('FAILED')
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position = position
        goal.pose.pose.orientation.w = 1.0  # no target orientation known -- identity is fine for arrival

        self.nav_client.send_goal_async(goal).add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f'{self.robot_name}: Nav2 rejected the goal.')
            self.finish_task('FAILED')
            return
        goal_handle.get_result_async().add_done_callback(self.on_nav_result)

    def on_nav_result(self, future):
        result = future.result()
        if result.status == NAV2_STATUS_SUCCEEDED:
            self.get_logger().info(f'{self.robot_name}: reached target {self.current_target_id}.')
            self.finish_task('COMPLETED')
        else:
            self.get_logger().warn(
                f'{self.robot_name}: navigation failed (status={result.status}).')
            self.finish_task('FAILED')

    def finish_task(self, outcome):
        if self.report_status_client.service_is_ready():
            req = ReportTaskStatus.Request()
            req.task_id = self.current_task_id
            req.robot_name = self.robot_name
            req.status = outcome
            self.report_status_client.call_async(req)
        else:
            self.get_logger().error('report_task_status not available, cannot report outcome.')

        self.current_task_id = -1
        self.current_target_id = -1
        self.busy = False
        self.task_id_pub.publish(Int32(data=-1))


def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
