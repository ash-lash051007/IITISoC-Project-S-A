import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class Robot1Explorer(Node):
    def __init__(self):
        super().__init__('robot1_explorer')
        # We target the robot1 namespace as per your launch file
        self._action_client = ActionClient(self, NavigateToPose, '/robot1/navigate_to_pose')
        
        # Waypoints chosen to cover the warehouse based on your .world file
        self.waypoints = [
            (25.0, 25.0), 
            (30.0, 20.0), 
            (15.0, 20.0), 
            (15.0, 10.0), 
            (25.0, 10.0)
        ]
        self.current_goal_idx = 0
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_goal_idx < len(self.waypoints):
            x, y = self.waypoints[self.current_goal_idx]
            self.get_logger().info(f'Sending goal: ({x}, {y})')
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = x
            goal_msg.pose.pose.position.y = y
            goal_msg.pose.pose.orientation.w = 1.0 
            
            self._action_client.wait_for_server()
            self._action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)
            self.current_goal_idx += 1
        else:
            self.get_logger().info('Exploration sequence complete!')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            return
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.get_logger().info('Goal reached, moving to next waypoint...')
        time.sleep(1) 
        self.send_next_goal()

def main(args=None):
    rclpy.init(args=args)
    explorer = Robot1Explorer()
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
