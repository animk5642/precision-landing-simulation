import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from pymavlink import mavutil  # NEW: Import PyMAVLink

class ArucoPerceptionNode(Node):
    def __init__(self):
        super().__init__('aruco_perception_node')
        
        # 1. Subscribers
        self.image_sub = self.create_subscription(
            Image, 
            '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image', 
            self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, 
            '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/camera_info', 
            self.info_callback, 10)
            
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # 2. ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

        # 3. NEW: Connect to SITL via PyMAVLink
        # We use port 14555 so we don't conflict with MAVProxy
        self.mav = mavutil.mavlink_connection('udp:127.0.0.1:14555')
        self.get_logger().info("Connected to MAVLink on port 14555")

    def info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        self.destroy_subscription(self.info_sub)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            corners, ids, rejected = self.detector.detectMarkers(cv_image)
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                
                # Solve PnP Math
                marker_size = 0.3 
                obj_points = np.array([
                    [-marker_size/2,  marker_size/2, 0],
                    [ marker_size/2,  marker_size/2, 0],
                    [ marker_size/2, -marker_size/2, 0],
                    [-marker_size/2, -marker_size/2, 0]
                ], dtype=np.float32)

                img_points = corners[0][0].astype(np.float32)
                success, rvec, tvec = cv2.solvePnP(obj_points, img_points, self.camera_matrix, self.dist_coeffs)
                
                if success:
                    # 1. Get Camera X, Y, Z (Meters)
                    x_cam, y_cam, z_cam = tvec.flatten()
                    
                    # 2. Map Camera Frame to Drone Body Frame
                    drone_forward = -y_cam  # Camera Y is down, flip for forward
                    drone_right = x_cam     # Camera X is right, matches drone right
                    drone_down = z_cam

                    # 3. Convert to Angles (Radians) for ArduPilot
                    angle_x = math.atan2(drone_forward, drone_down)
                    angle_y = math.atan2(drone_right, drone_down)
                    
                    self.get_logger().info(f"Distances -> F: {drone_forward:.2f}, R: {drone_right:.2f}, D: {drone_down:.2f}")
                    self.get_logger().info(f"Angles -> X: {angle_x:.3f}, Y: {angle_y:.3f}")

                    # 4. Send the LANDING_TARGET message to ArduPilot
                    self.mav.mav.landing_target_send(
                        int(self.get_clock().now().nanoseconds / 1000), # timestamp
                        0,                                              # target_num
                        mavutil.mavlink.MAV_FRAME_BODY_NED,             # frame (Drone Body Frame)
                        angle_x,                                        # angle_x (Forward/Back)
                        angle_y,                                        # angle_y (Right/Left)
                        drone_down,                                     # distance
                        0, 0, 0, 0, 0, 0, 0, 0, 0                       # unused fields
                    )

            cv2.imshow("Aruco Detection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()