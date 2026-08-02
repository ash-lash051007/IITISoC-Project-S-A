import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
import time

# TF2 imports
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException

class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector_temp')

        self.declare_parameter('robot_name', 'robot1')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        
        self.waste_type_map = {
            0: "recyclable", 1: "recyclable", 2: "recyclable", 3: "recyclable",
            4: "hazardous",  5: "hazardous",  6: "hazardous",  7: "hazardous",
            8: "general",    9: "general",    10: "general",   11: "general",
        }

        self.get_logger().info(f'Starting detector for: {self.robot_name}')

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        # ── TF2 Setup ────────────────────────────────────────────
        # Buffer stores all incoming transforms
        # TransformListener fills the buffer automatically
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera optical frame — where our detection coordinates come from
        self.camera_frame = f'{self.robot_name}/camera_rgb_optical_frame'

        # Target frame — where we want coordinates to end up
        self.target_frame = 'map'

        self.create_subscription(
            CameraInfo,
            f'/{self.robot_name}/camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.create_subscription(
            Image,
            f'/{self.robot_name}/camera/image_raw',
            self.image_callback,
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            f'/{self.robot_name}/camera/image_aruco_annotated',
            10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            f'/{self.robot_name}/aruco_detections/pose',
            10
        )

        self.log_pub = self.create_publisher(
            String,
            f'/{self.robot_name}/aruco_detections/log',
            10
        )

        self.get_logger().info(f'ArUco Detector ready for {self.robot_name}')

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def transform_to_map(self, x, y, z, stamp):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            from geometry_msgs.msg import Pose
            from tf2_geometry_msgs import do_transform_pose

            camera_pose = Pose()
            camera_pose.position.x = float(x)
            camera_pose.position.y = float(y)
            camera_pose.position.z = float(z)
            camera_pose.orientation.w = 1.0
            camera_pose.orientation.x = 0.0
            camera_pose.orientation.y = 0.0
            camera_pose.orientation.z = 0.0

            map_pose = do_transform_pose(camera_pose, transform)

            return (
                map_pose.position.x,
                map_pose.position.y,
                map_pose.position.z
            )

        except (LookupException, ConnectivityException,
                ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF2 transform failed: {e}',
                throttle_duration_sec=3.0
            )
            return None

    def publish_detection(self, marker_id, map_x, map_y, map_z, distance, stamp):
        """
        Publishes detection data using MAP coordinates.
        """
        waste_type = self.waste_type_map.get(int(marker_id), "general")
        # PoseStamped in map frame — ready for Nav2
        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'   # now in map frame
        pose_msg.pose.position.x = float(map_x)
        pose_msg.pose.position.y = float(map_y)
        pose_msg.pose.position.z = float(map_z)
        pose_msg.pose.orientation.w = 1.0
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        self.pose_pub.publish(pose_msg)

        # JSON string with map coordinates for database
        detection_data = {
            'marker_id': int(marker_id),
            'waste_type': waste_type,
            'map_x': round(float(map_x), 3),
            'map_y': round(float(map_y), 3),
            'map_z': round(float(map_z), 3),
            'distance': round(float(distance), 3),
            'robot': self.robot_name,
            'timestamp': time.time()
        }
        log_msg = String()
        log_msg.data = json.dumps(detection_data)
        self.log_pub.publish(log_msg)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge Error (Incoming): {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            for i, marker_id in enumerate(ids.flatten()):

                marker_corners = corners[i]
                marker_corners_2d = corners[i][0]

                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    marker_corners,
                    0.5,
                    self.camera_matrix,
                    self.dist_coeffs
                )

                # Camera frame coordinates
                cam_x = tvec[0][0][0]
                cam_y = tvec[0][0][1]
                cam_z = tvec[0][0][2]
                distance = float(np.sqrt(cam_x**2 + cam_y**2 + cam_z**2))

                cv2.drawFrameAxes(
                    frame, self.camera_matrix, self.dist_coeffs,
                    rvec, tvec, 0.1
                )

                # ── TF2 Transform to Map Frame ───────────────────
                map_coords = self.transform_to_map(
                    cam_x, cam_y, cam_z,
                    msg.header.stamp
                )

                if map_coords is not None:
                    map_x, map_y, map_z = map_coords

                    # Draw label with MAP coordinates
                    center_x = int(np.mean(marker_corners_2d[:, 0]))
                    center_y = int(np.mean(marker_corners_2d[:, 1]))

                    text = f'ID:{marker_id} ({map_x:.2f},{map_y:.2f})'
                    text_size = cv2.getTextSize(
                        text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                    text_x = center_x - text_size[0] // 2
                    text_y = center_y - 20

                    cv2.rectangle(
                        frame,
                        (text_x - 5, text_y - text_size[1] - 5),
                        (text_x + text_size[0] + 5, text_y + 5),
                        (0, 0, 0), -1
                    )
                    cv2.putText(
                        frame, text,
                        (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2
                    )

                    # Log world coordinates to terminal
                    self.get_logger().info(
                        f'[{self.robot_name}] ID: {marker_id} | '
                        f'Map: ({map_x:.2f}, {map_y:.2f}) | '
                        f'Distance: {distance:.2f}m'
                    )

                    # Publish with map coordinates
                    self.publish_detection(
                        marker_id, map_x, map_y, map_z,
                        distance, msg.header.stamp
                    )
                else:
                    # TF2 failed — fall back to camera coordinates
                    center_x = int(np.mean(marker_corners_2d[:, 0]))
                    center_y = int(np.mean(marker_corners_2d[:, 1]))

                    text = f'ID:{marker_id} D:{distance:.2f}m'
                    text_size = cv2.getTextSize(
                        text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                    text_x = center_x - text_size[0] // 2
                    text_y = center_y - 20

                    cv2.rectangle(
                        frame,
                        (text_x - 5, text_y - text_size[1] - 5),
                        (text_x + text_size[0] + 5, text_y + 5),
                        (0, 0, 0), -1
                    )
                    cv2.putText(
                        frame, text,
                        (text_x, text_y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 255), 2
                    )

                    self.get_logger().warn(
                        f'[{self.robot_name}] ID: {marker_id} | '
                        f'TF2 unavailable, camera coords: '
                        f'x={cam_x:.2f} y={cam_y:.2f} z={cam_z:.2f}'
                    )

        else:
            self.get_logger().info(
                f'[{self.robot_name}] No markers detected',
                throttle_duration_sec=3.0
            )

        frame_resized = cv2.resize(frame, (640, 480))
        cv2.imshow(f'ArUco Detection - {self.robot_name}', frame_resized)
        cv2.waitKey(1)

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.image_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f'CvBridge Error (Outgoing): {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
