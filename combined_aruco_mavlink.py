#!/usr/bin/env python3
"""
ArUco precision-landing node (ROS 2 + Gazebo + ArduPilot SITL)

What it does:
  1. Reads the gimbal camera image + camera_info from ROS 2.
  2. Detects the ArUco marker and gets its pose with solvePnP.
  3. Sends MAVLink LANDING_TARGET to ArduPilot so LAND mode steers to the marker.

Camera frame (OpenCV): X = right in image, Y = down in image, Z = forward along lens.
LANDING_TARGET wants the angular offset of the target from the image centre:
  angle_x = +right in the image, angle_y = +down in the image.
ArduPilot converts these to its body frame itself (camera pointing down,
top of image = drone nose). So we send image-style angles, NOT forward/right.

Before running, in MAVProxy:
    output add 127.0.0.1:14560
    param set PLND_ENABLED 1
    param set PLND_TYPE 1
    (reboot SITL after changing PLND params)
"""

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from pymavlink import mavutil
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

CAM_BASE = ('/world/iris_runway/model/iris_with_gimbal/model/gimbal'
            '/link/pitch_link/sensor/camera')


class ArucoPrecLandNode(Node):
    def __init__(self):
        super().__init__('aruco_perception_node')

        # ---- Parameters (change with: --ros-args -p name:=value) ----
        self.declare_parameter('marker_size', 0.3)        # metres, must match the SDF
        self.declare_parameter('marker_id', -1)           # -1 = accept any ID
        self.declare_parameter('mav_url', 'udpin:127.0.0.1:14560')
        self.declare_parameter('show_window', True)
        self.declare_parameter('flip_x', False)           # use if drone drifts wrong way left/right
        self.declare_parameter('flip_y', False)           # use if drone drifts wrong way fwd/back
        self.declare_parameter('swap_xy', False)          # use if camera is rotated 90 deg

        self.marker_size = self.get_parameter('marker_size').value
        self.marker_id = self.get_parameter('marker_id').value
        self.show_window = self.get_parameter('show_window').value
        self.flip_x = self.get_parameter('flip_x').value
        self.flip_y = self.get_parameter('flip_y').value
        self.swap_xy = self.get_parameter('swap_xy').value

        # ---- ROS subscribers ----
        self.image_sub = self.create_subscription(
            Image, CAM_BASE + '/image', self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, CAM_BASE + '/camera_info', self.info_callback, 10)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # ---- ArUco ----
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, cv2.aruco.DetectorParameters())

        half = self.marker_size / 2.0
        self.obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)

        # ---- MAVLink ----
        # MAVProxy must have: output add 127.0.0.1:14560
        url = self.get_parameter('mav_url').value
        self.mav = mavutil.mavlink_connection(url)
        self.get_logger().info(f'Waiting for MAVLink heartbeat on {url} ...')
        self.mav.wait_heartbeat(timeout=15)
        self.get_logger().info(
            f'Heartbeat from system {self.mav.target_system}, '
            f'component {self.mav.target_component}')

        self.last_log = 0.0

    def info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d) if len(msg.d) else np.zeros(5)
        self.get_logger().info('Camera info received')
        self.destroy_subscription(self.info_sub)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            corners, ids, _ = self.detector.detectMarkers(frame)

            if ids is not None:
                idx = self.pick_marker(ids)
                if idx is not None:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    img_points = corners[idx][0].astype(np.float32)
                    ok, rvec, tvec = cv2.solvePnP(
                        self.obj_points, img_points,
                        self.camera_matrix, self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE)

                    if ok:
                        self.handle_pose(tvec.flatten())

            if self.show_window:
                cv2.imshow('Aruco Detection', frame)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def pick_marker(self, ids):
        """Return the index of the marker we want, or None."""
        flat = ids.flatten()
        if self.marker_id < 0:
            return 0
        matches = np.where(flat == self.marker_id)[0]
        return int(matches[0]) if len(matches) else None

    def handle_pose(self, tvec):
        x_cam, y_cam, z_cam = tvec  # right, down, depth (metres)

        # Angular offsets from image centre (radians)
        angle_x = math.atan2(x_cam, z_cam)   # +right in image
        angle_y = math.atan2(y_cam, z_cam)   # +down in image

        if self.swap_xy:
            angle_x, angle_y = angle_y, angle_x
        if self.flip_x:
            angle_x = -angle_x
        if self.flip_y:
            angle_y = -angle_y

        distance = float(np.linalg.norm(tvec))

        # Apparent size of the marker, in radians
        size_rad = 2.0 * math.atan2(self.marker_size / 2.0, z_cam)

        self.send_landing_target(angle_x, angle_y, distance, size_rad)

        # Human-friendly log, once per second
        now = time.time()
        if now - self.last_log > 1.0:
            self.last_log = now
            forward = -y_cam
            right = x_cam
            self.get_logger().info(
                f'Marker: forward={forward:+.2f} m, right={right:+.2f} m, '
                f'down={z_cam:.2f} m | angle_x={angle_x:+.3f}, '
                f'angle_y={angle_y:+.3f} rad')

    def send_landing_target(self, angle_x, angle_y, distance, size_rad):
        self.mav.mav.landing_target_send(
            int(time.time() * 1e6),                       # time_usec
            0,                                            # target_num
            mavutil.mavlink.MAV_FRAME_BODY_FRD,           # frame
            angle_x,                                      # angle_x (rad)
            angle_y,                                      # angle_y (rad)
            distance,                                     # distance (m)
            size_rad,                                     # size_x (rad)
            size_rad,                                     # size_y (rad)
            0.0, 0.0, 0.0,                                # x, y, z (unused)
            [1.0, 0.0, 0.0, 0.0],                         # q (unused)
            mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
            0)                                            # position_valid = 0


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPrecLandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()