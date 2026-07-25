import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time
import math


class PatrolRobot(Node):

    def __init__(self, robot_name, waypoints):
        super().__init__(f'{robot_name}_patrol')
        self.robot_name = robot_name
        self.waypoints = waypoints
        self.current_waypoint = 0
        self.client = ActionClient(
            self,
            NavigateToPose,
            f'/{robot_name}/navigate_to_pose'
        )
        self.get_logger().info(
            f'{robot_name} patrol started — '
            f'{len(waypoints)} waypoints'
        )

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = (
            math.sin(yaw / 2.0)
        )
        goal_msg.pose.pose.orientation.w = (
            math.cos(yaw / 2.0)
        )

        self.get_logger().info(
            f'[{self.robot_name}] -> ({x:.1f}, {y:.1f})'
        )

        self.client.wait_for_server()
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                f'Goal ({x},{y}) rejected — skipping'
            )
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info(
            f'[{self.robot_name}] reached ({x:.1f}, {y:.1f})'
        )

    def patrol(self):
        while rclpy.ok():
            x, y, yaw = (
                self.waypoints[self.current_waypoint]
            )
            self.send_goal(x, y, yaw)
            time.sleep(0.5)
            self.current_waypoint = (
                self.current_waypoint + 1
            ) % len(self.waypoints)


def main():
    rclpy.init()

    robot1_waypoints = [

        # ── Start position ──
        (-2.0, -2.0,  0.0),

        # ── South wall sweep (left half) ──
        # Face north to see markers on south wall
        (-3.0, -5.5,  1.57),   # Near M4 (-3.0,-5.9)
        (-1.5, -5.5,  1.57),   # Between M4 and M2
        (-0.5, -5.5,  1.57),   # Near M2 (0.0,-6.9), stay left of x=0

        # ── West wall sweep ──
        # Face east to see markers on west wall
        (-5.5, -3.0,  0.0),    # Near M5 (-5.9,-3.0)
        (-5.5,  0.0,  0.0),    # Mid west wall
        (-5.5,  2.0,  0.0),    # Near M0 (-6.9,2.0)
        (-5.5,  5.5,  0.0),    # Top west corner area

        # ── North wall left half ──
        # Face south along north wall
        (-4.0,  5.5, -1.57),   # North wall left
        (-2.0,  5.5, -1.57),   # North wall centre-left
        (-0.5,  5.5, -1.57),   # North wall near centre, stay left

        # ── Inner corridor sweep ──
        # x=-3.5 gives 1.5m clearance from shelves at x=-5
        (-3.5,  3.5,  0.0),    # Inner corridor top
        (-3.5,  0.0,  0.0),    # Inner corridor middle
        (-3.5, -3.5,  0.0),    # Inner corridor bottom

        # ── Return to start ──
        (-2.0, -2.0,  0.0),
    ]

    patrol = PatrolRobot('robot1', robot1_waypoints)
    patrol.patrol()
    rclpy.shutdown()


if __name__ == '__main__':
    main()