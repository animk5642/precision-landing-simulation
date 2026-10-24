import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoPerceptionNode(Node):
    def __init__(self):
        super().__init__('aruco_perception_node')
        
        # Subscriber for the image
        self.image_sub = self.create_subscription(
            Image, 
            '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image', 
            self.image_callback, 10)
            
        # Subscriber for the camera info (calibration)
        self.info_sub = self.create_subscription(
            CameraInfo, 
            '/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/camera_info', 
            self.info_callback, 10)
            
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())

    def info_callback(self, msg):
        # Extract the 3x3 Intrinsic Matrix (K) and Distortion (D)
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        # We only need this once, so we can destroy the subscriber to save CPU
        self.destroy_subscription(self.info_sub)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().info("Waiting for camera info...")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            corners, ids, rejected = self.detector.detectMarkers(cv_image)
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                
                # --- THE MATH BEGINS HERE ---
                # Marker size in meters (e.g., 0.1 meters = 10cm)
                marker_size = 0.3
                
                # 3D points of the marker corners in real life (Z=0 because it's flat)
                obj_points = np.array([
                    [-marker_size/2,  marker_size/2, 0],
                    [ marker_size/2,  marker_size/2, 0],
                    [ marker_size/2, -marker_size/2, 0],
                    [-marker_size/2, -marker_size/2, 0]
                ], dtype=np.float32)

                # 2D points from the camera
                img_points = corners[0][0].astype(np.float32)

                # SOLVE PNP
                success, rvec, tvec = cv2.solvePnP(
                    obj_points, img_points, self.camera_matrix, self.dist_coeffs
                )
                
                if success:
                    # tvec contains the X, Y, Z distance in meters
                    x, y, z = tvec.flatten()
                    self.get_logger().info(f"Distance: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m")

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