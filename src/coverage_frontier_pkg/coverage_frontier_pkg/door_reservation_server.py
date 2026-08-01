#!/usr/bin/env python3
"""
door_reservation_server.py

Trivial mutex per door. Robots normally stay within their own room, so
this only matters during reallocation (a robot finishing early crosses
into another room) or the rare case two robots' rooms share heavy
traffic through one door. First robot to request a door gets it;
second robot is told "not granted" and should wait/retry.
"""

import rclpy
from rclpy.node import Node
from coverage_frontier_interfaces.srv import ReserveDoor


class DoorReservationServer(Node):
    def __init__(self):
        super().__init__("door_reservation_server")
        self.locks = {}  # door_name -> robot_id holding it
        self.create_service(ReserveDoor, "reserve_door", self._on_reserve)
        self.get_logger().info("DoorReservationServer up.")

    def _on_reserve(self, request, response):
        door = request.door_name
        robot = request.robot_id

        if request.release:
            if self.locks.get(door) == robot:
                del self.locks[door]
                self.get_logger().info(f"{robot} released door '{door}'")
            response.granted = True
            return response

        holder = self.locks.get(door)
        if holder is None or holder == robot:
            self.locks[door] = robot
            response.granted = True
            self.get_logger().info(f"{robot} granted door '{door}'")
        else:
            response.granted = False
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DoorReservationServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
